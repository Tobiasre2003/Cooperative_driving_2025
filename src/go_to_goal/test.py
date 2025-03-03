#!/bin/env python3

import rospy
from gv_client.msg import GulliViewPosition
from roswifibot.msg import Status
import sys
import socket
import threading
import select
from gv_client.msg import LaptopSpeed
from std_msgs.msg import Header
import math



class Point:
    def __init__(self,x:float, y:float):
        self.x = x
        self.y = y

    def __str__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
    def __repr__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
    def distance_between_points(self, point) -> float:
        return math.sqrt((self.x-point.x)**2 + (self.y-point.y)**2)
    
    def get_moved_point(self, point):
        return Point(self.x+point.x, self.y+point.y)
    

class Vector:
    def __init__(self,x:float, y:float):
        self.x = x
        self.y = y
    
    def __repr__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
    def abs(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def dot_product(self, vector) -> float:
        return self.x * vector.x + self.y * vector.y

    def scalar_projection(self, vector) -> float:
        length = self.abs() 
        if length == 0: length = 1
        return self.dot_product(vector) / length
    
    def vector_multiplication(self, coefficient:float):
        return Vector(self.x*coefficient, self.y*coefficient)
    
    def get_orthogonal_vector(self):
        return Vector(self.y, -self.x)
    
    def get_unit_vector(self):
        length = self.abs() 
        if length == 0: length = 1
        return self.vector_multiplication(1/length)
    
    def to_point(self) -> Point: 
        return Point(self.x, self.y)
        
    def vector_projection(self, vector):
        return self.vector_multiplication(self.scalar_projection(vector))

    
class Object:
    def __init__(self, pos:Point, theta:float, borders:list[Point] = [], velocity_vector:Vector = Vector(0,0)):
        self.pos = pos
        self.direction = theta
        self.borders = borders
        self.velocity_vector = velocity_vector.get_unit_vector()
        
    def update(self, pos:Point, theta:float):
        self.pos = pos
        self.direction = theta
        
    def rotation_matrix(self, sign:int = 1) -> tuple[Vector]:
        return (Vector(math.cos(sign*self.direction), math.sin(sign*self.direction)), Vector(-math.sin(sign*self.direction), math.cos(sign*self.direction)))
    
    def get_global_velocity_vector(self):
        rotation_matrix = self.rotation_matrix(-1)
        x = rotation_matrix[0].dot_product(self.velocity_vector)
        y = rotation_matrix[1].dot_product(self.velocity_vector)
        return Vector(x, y).get_unit_vector()
    
    def point_from_local_to_global(self, local_point:Point) -> Point:
        rotation_matrix = self.rotation_matrix(-1)
        x = rotation_matrix[0].dot_product(local_point) + self.pos.x
        y = rotation_matrix[1].dot_product(local_point) + self.pos.y
        return Point(x, y)
    
    def from_local_to_global(self) -> list[Point]:
        borders = []
        for local_point in self.borders:
            borders.append(self.point_from_local_to_global(local_point))
        return borders
    
    def point_from_globl_to_local(self, global_point:Point) -> Point:
        rotation_matrix = self.rotation_matrix()
        x = rotation_matrix[0].dot_product(global_point) - rotation_matrix[0].dot_product(self.pos)
        y = rotation_matrix[1].dot_product(global_point) - rotation_matrix[1].dot_product(self.pos)
        return Point(x, y)
    
    def from_globl_to_local(self, global_borders:list[Point]) -> list[Point]:
        borders = []
        for global_point in global_borders:
            borders.append(self.point_from_globl_to_local(global_point))
        return borders
    
    def get_outer_points(self):
        side_min_point = self.pos
        side_max_point = self.pos
        front_max_point = self.pos
        side_min_dist = math.inf
        side_max_dist = -math.inf
        front_max_dist = -math.inf

        for point in self.borders:
            side_dist = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point)
            front_dist = self.velocity_vector.get_unit_vector().dot_product(point)
            
            if side_dist>side_max_dist: 
                side_max_point = point
                side_max_dist = side_dist
            if side_dist<side_min_dist: 
                side_min_point = point
                side_min_dist = side_dist
            if front_dist>front_max_dist:
                front_max_dist = front_dist
                front_max_point = point
        
        return side_min_point, side_max_point, self.velocity_vector.vector_projection(front_max_point).to_point()
    
    def collision_course(self, obj):
        p1, p2, front_point = self.get_outer_points()
        sign = 0
        closest_distance = math.inf
        collision = False
        
        for point in self.from_globl_to_local(obj.from_local_to_global()):
            dist_from_vector_1 = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point.get_moved_point(p1))
            dist_from_vector_2 = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point.get_moved_point(p2))
 
            if dist_from_vector_1*dist_from_vector_2>0: 
                if sign==0: sign = math.copysign(1,dist_from_vector_1)
                if sign != math.copysign(1,dist_from_vector_1):
                    if self.velocity_vector.dot_product(point) >= 0: collision = True
                
            if (dist_from_vector_1<=0 and dist_from_vector_2>=0): 
                if self.velocity_vector.dot_product(point) >= 0: collision = True
            
            if collision:    
                obj_collision_point = self.velocity_vector.vector_projection(point).to_point()
                distance = obj_collision_point.distance_between_points(front_point)
                if distance < closest_distance: closest_distance = distance
                        
        return collision, closest_distance
    
    def crossing_vector(self, self_point:Point, obj_point:Point, self_vel:Vector, obj_vel:Vector):
        try:
            self_dist = (obj_point.y-self_point.y+obj_vel.y*((self_point.x-obj_point.x)/obj_vel.x))/(self_vel.y-(obj_vel.y*self_vel.x)/obj_vel.x)
            return self_vel.vector_multiplication(self_dist).to_point().get_moved_point(self_point) 
        except ZeroDivisionError:
            return False
     
    def moving_collision_course(self, obj):
        self_p1, self_p2,_ = self.get_outer_points()
        obj_p1, obj_p2,_ = obj.get_outer_points()
        
        self_p1 = self.point_from_local_to_global(self_p1)
        self_p2 = self.point_from_local_to_global(self_p2)
        obj_p1 = obj.point_from_local_to_global(obj_p1)
        obj_p2 = obj.point_from_local_to_global(obj_p2)
        
        self_vel = self.get_global_velocity_vector()
        obj_vel = obj.get_global_velocity_vector()
    
        return self.crossing_vector(self_p1, obj_p2, self_vel, obj_vel), self.crossing_vector(self_p2, obj_p1, self_vel, obj_vel)
        

     

    
class Bot:
    def __init__(self, id:int):
        self.id = id
        self.left_speed = 0
        self.right_speed = 0
        self.absolute_speed = 0
        self.last_speed_update = None
        self.point = None
        self.theta = None
        
        self.last_update = rospy.get_time()
        self.enter = False
        
        self.current_lane = ""
        self.next_lane = ""
        
        self.mean_time_to_intersection = None
        
        self.acceleration = 0
        
        self.obj = Object(self.point, self.theta, [Point(210,160),Point(-210,160),Point(-210,-160),Point(210,-160)], (0,1))

    def update_pos(self, point, theta):
        self.point = point
        self.theta = theta
        self.obj.update(self.point, self.theta)
        
    def add_speed(self, speed:float):
        time = rospy.get_time()
        if not self.last_speed_update is None:
            self.acceleration = (speed-self.absolute_speed)/(time-self.last_speed_update) # [m/s^2]
        self.last_speed_update = time 
        self.absolute_speed = speed
                
    def __repr__(self):
        return f'(Id: {self.id}, Left speed: {self.left_speed}, Right speed: {self.right_speed}, Absolute speed: {self.absolute_speed}, Point: {self.point})'


class Receiver:
    def __init__(self):
        rospy.Subscriber('gv_positions', GulliViewPosition, self.positions)
        rospy.Subscriber('status', Status, self.status)
        
    def positions(self, position_msg):
        id = position_msg.tagId
        x = position_msg.x
        y = position_msg.y
        theta = position_msg.theta
        self.p = Point(x, y)
        if id in bots:
            bots[id].update_pos(self.p, theta)
        else: 
            bots[id] = Bot(id)
            bots[id].update_pos(self.p, theta)

    def status(self, status_msg):
        bots[my_id].left_speed = status_msg.speed_front_left/2
        bots[my_id].right_speed = status_msg.speed_front_right/2
        bots[my_id].add_speed((bots[my_id].left_speed + bots[my_id].left_speed)/2) # Average speed forward, divided by two to match speed on cmdvel

def listen(run_flag, adress, port, d):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (adress, port)
    sock.bind(server_address)
    while run_flag.is_set():
        try:
            ready, _, _ = select.select([sock], [], [], 1)
            if ready:
                data, sender_adress = sock.recvfrom(4096)
                d.data = data.decode()
                d.adress = sender_adress
                d.update()
        except socket.timeout:
            break

    sock.close()
        
def send(adress, port, msg):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg.encode(), (adress, port))
    sock.close()

class Speed:
    def __init__(self):
        self.publisher = rospy.Publisher('gv_laptop', LaptopSpeed, queue_size=10)
    
    def create_msg(self, speed, restart = False):
        header = Header()
        header.stamp = rospy.Time.now()
        return LaptopSpeed(header=header, tag_id=my_id, speed=speed, restart=restart)
    
    def setSpeed(self, speed):
        msg = self.create_msg(speed)
        self.publisher.publish(msg)


class Data:
    data = ""
    adress = None
    
    def isfloat(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False

    def to_list(self, string = ""):
        if string == "": string = self.data
        string = string[1:-1]
        str_element = ""
        arr = []
        block = 0
        for j in range(len(string)):
            if string[j]=="[": block+=1
            if string[j]=="]": block-=1
            if string[j] == "," and block == 0:
                arr.append(str_element)
                str_element="" 
                continue
            str_element+=string[j]
        arr.append(str_element)
        for i in range(len(arr)):
            element = arr[i]
            if i != 0:element=element[1:]
            if "[" in element: element = self.to_list(element)
            elif element.isdigit(): element = int(element)
            elif self.isfloat(element): element = float(element)
            elif element == "False": element = False
            elif element == "True": element = True
            else: element = element[1:-1]
            arr[i] = element
        return arr
    
    def update(self):
        data_list = self.to_list()
        id = data_list[0]
        msg_type = data_list[1]
        time = rospy.get_time()
        
        if msg_type == "ENTER" : 
            bots[id].enter = True
            bots[id].current_lane = data_list[2]
            bots[id].next_lane = data_list[3]
            bots[id].mean_time_to_intersection = data_list[4]
            bots[id].last_update = time
        
        elif msg_type == "EXIT" : 
            bots[id].enter = False
            bots[id].next_lane = data_list[2]
            bots[id].last_update = time
        
        elif msg_type == "HB" : 
            state = data_list[2]
            bots[id].point.x = state[0]
            bots[id].point.y = state[1]
            bots[id].absolute_speed = state[2]
            bots[id].acceleration = state[3]
            bots[id].last_update = time
        


        

if __name__ == '__main__': 
    try:
        
        d = Data()
        s = Speed()
        
        run_flag = threading.Event()
        run_flag.set()
        my_id = int(sys.argv[1])
        bots = {my_id:Bot(my_id)}
        
        ip = {4:'192.168.50.102',5:'192.168.50.103'}
        
        rospy.init_node('test', anonymous=True)
        rospy.loginfo("Starting test node")
        
        node = Receiver()
        
        t = threading.Thread(target=listen, args=(run_flag, ip[my_id], 2020, d))
        t.start()
        s.setSpeed(0)
        while not rospy.is_shutdown():
            if not d.adress is None: 
                print(d.to_list(), d.adress[0])
                send(d.adress[0], 2020, d.data)
                exit()
    
            if bots[my_id].absolute_speed != 0:
                print(bots[my_id].absolute_speed,bots[my_id].right_speed,bots[my_id].left_speed)
                
    except KeyboardInterrupt:
        run_flag.clear()
        t.join()
    finally:
        run_flag.clear()
        t.join()
        

            

