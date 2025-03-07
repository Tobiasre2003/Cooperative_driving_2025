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
from geometry_msgs.msg import Polygon, Point32
import math
from go_to_goal import SPEED

rospy.init_node('cooperative_maneuver_control', anonymous=True)
rospy.loginfo("Starting cooperative_maneuver_control node")


time_step = 1
last_time_step = rospy.get_time()
def next_time_step() -> bool:
    if rospy.get_time() > last_time_step + time_step:
        last_time_step = rospy.get_time()
        return True
    return False




class Point:
    def __init__(self,x:float, y:float):
        self.x = x
        self.y = y

    def __str__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
    def __repr__(self):
        return f'(X: {self.x}, Y: {self.y})'
    
    def __add__(self, point):
        try: return Point(self.x+point.x, self.y+point.y)
        except: return Point(self.x+point, self.y+point)
    
    def __sub__(self, point):
        try: return Point(self.x-point.x, self.y-point.y)
        except: return Point(self.x-point, self.y-point)
    
    def __mul__(self, point):
        try: return Point(self.x*point.x, self.y*point.y)
        except: return Point(self.x*point, self.y*point)
    
    def distance_between_points(self, point) -> float:
        if point == None: return None
        return math.sqrt((self.x-point.x)**2 + (self.y-point.y)**2)
    
    def distance_between_points_in_list(self, list:list) -> float:
        try:
            prev_point = self
            tot_dist = 0
            for point in list:
                tot_dist += prev_point.distance_between_points(point)
                prev_point = point
            return tot_dist
        except: 
            return None

class Vector:
    def __init__(self,x:float, y:float):
        self.x = x
        self.y = y
    
    def __repr__(self):
        return f'(X: {self.x}, Y: {self.y})'

    def __add__(self, vector):
        try: return Vector(self.x+vector.x, self.y+vector.y)
        except: return Vector(self.x+vector, self.y+vector)
    
    def __sub__(self, vector):
        try: return Vector(self.x-vector.x, self.y-vector.y)
        except: return Vector(self.x-vector, self.y-vector)
    
    def __mul__(self, vector):
        try: return Vector(self.x*vector.x, self.y*vector.y)
        except: return Vector(self.x*vector, self.y*vector)

    def abs(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def dot_product(self, vector) -> float:
        return self.x * vector.x + self.y * vector.y

    def scalar_projection(self, vector) -> float:
        length = self.abs() 
        if length == 0: length = 1
        return self.dot_product(vector) / length
    
    def get_orthogonal_vector(self):
        return Vector(self.y, -self.x)
    
    def get_unit_vector(self):
        length = self.abs() 
        if length == 0: length = 1
        return self*(1/length)
    
    def to_point(self) -> Point: 
        return Point(self.x, self.y)
        
    def vector_projection(self, vector):
        return self*self.scalar_projection(vector)

    
class Object:
    def __init__(self, pos:Point, theta:float, borders:list = [], velocity_vector:Vector = Vector(0,0)):
        self.pos = pos
        self.direction = theta
        self.borders = borders
        self.velocity_vector = velocity_vector.get_unit_vector()
        
    def update(self, pos:Point, theta:float):
        self.pos = pos
        self.direction = theta
        
    def rotation_matrix(self, sign:int = 1) -> tuple:
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
    
    def from_local_to_global(self) -> list:
        borders = []
        for local_point in self.borders:
            borders.append(self.point_from_local_to_global(local_point))
        return borders
    
    def point_from_globl_to_local(self, global_point:Point) -> Point:
        rotation_matrix = self.rotation_matrix()
        x = rotation_matrix[0].dot_product(global_point) - rotation_matrix[0].dot_product(self.pos)
        y = rotation_matrix[1].dot_product(global_point) - rotation_matrix[1].dot_product(self.pos)
        return Point(x, y)
    
    def from_globl_to_local(self, global_borders:list) -> list:
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
            dist_from_vector_1 = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point+p1)
            dist_from_vector_2 = self.velocity_vector.get_orthogonal_vector().get_unit_vector().dot_product(point+p2)
 
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
            if not obj_vel.x == 0:
                self_dist = (obj_point.y-self_point.y+obj_vel.y*((self_point.x-obj_point.x)/obj_vel.x))/(self_vel.y-(obj_vel.y*self_vel.x)/obj_vel.x)
                return (self_vel*self_dist).to_point() + self_point 
            elif not self_vel.x == 0:
                self_dist = (self_point.y-obj_point.y+self_vel.y*((obj_point.x-self_point.x)/self_vel.x))/(obj_vel.y-(self_vel.y*obj_vel.x)/self_vel.x)
                return (obj_vel*self_dist).to_point() + obj_point 
        except ZeroDivisionError:
            return False
     
    def moving_collision_course(self, obj):
        self_p1, self_p2, front_point = self.get_outer_points()
        obj_p1, obj_p2, _ = obj.get_outer_points()
        
        self_p1 = self.point_from_local_to_global(self_p1)
        self_p2 = self.point_from_local_to_global(self_p2)
        obj_p1 = obj.point_from_local_to_global(obj_p1)
        obj_p2 = obj.point_from_local_to_global(obj_p2)
        
        self_vel = self.get_global_velocity_vector()
        obj_vel = obj.get_global_velocity_vector()
    
        crossing_point_1 = self.crossing_vector(self_p1, obj_p2, self_vel, obj_vel)
        crossing_point_2 = self.crossing_vector(self_p2, obj_p1, self_vel, obj_vel)
        
        return crossing_point_1, front_point.distance_between_points(crossing_point_1), crossing_point_2, front_point.distance_between_points(crossing_point_2)
        
    def global_point_in_object(self, global_point:Point):
        local_point = self.point_from_globl_to_local(global_point)
        prev_point = self.borders[-1]
        sign = 0
        for point in self.borders:
            lp = local_point - prev_point 
            vector = Vector(point.x-prev_point.x, point.y-prev_point.y).get_orthogonal_vector()
            dist = vector.scalar_projection(lp)
            prev_point = point
            if sign == 0: sign = math.copysign(1, dist)
            elif not sign == math.copysign(1, dist): return False
        return True and not sign == 0
        

    
class Bot:
    def __init__(self, id:int):
        self.id = id
        self.left_speed = 0
        self.right_speed = 0
        self.absolute_speed = 0
        self.last_speed_update = None
        self.acceleration = 0
        self.point = Point(0,0)
        self.theta = 0
        self.last_update = rospy.get_time()
        self.obj = Object(self.point, self.theta, [Point(210,160),Point(-210,160),Point(-210,-160),Point(210,-160)], Vector(0,1))
        self.path = []
        
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
        return f'(Id: {self.id}, Absolute speed: {self.absolute_speed}, Point: {self.point})'
    
    def loss_of_signal(self):
        return rospy.get_time() - self.last_update > time_step + 1
    
    
                

class Intersection_section:
    def __init__(self, p1:Point, p2:Point, n, theta):
        dx = abs(p2.x-p1.x)
        dy = abs(p2.y-p1.y)
        self.n = n
        self.center_point = p1 + Point(dx/4 + int(n%2)*dx/2, dy/4 + int(math.floor(n/2))*dy/2)
        self.obj = Object(self.center_point, theta, [Point(-dx/4,-dy/4),Point(-dx/4,dy/4),Point(dx/4,dy/4),Point(dx/4,-dy/4)])
        self.claimed = None

    def get_path_dist(self, bot:Bot):
        entry_outside = None
        entry_inside = None
        entry_tot_dist_list = []
        exit_outside = None
        exit_inside = None
        exit_tot_dist_list = []

        bot_start_dist = 0
        
        for i in range(len(bot.path)):
            point = bot.path[i]
            if self.obj.global_point_in_object(point): 
                entry_inside = point
                exit_tot_dist_list = entry_tot_dist_list.copy()
                for point in bot.path[i:]:
                    if not self.obj.global_point_in_object(point): 
                        exit_outside = point
                        break
                    else: 
                        exit_tot_dist_list.append(exit_inside.distance_between_points(point))
                        exit_inside = point
                break
            else: 
                if entry_outside == None: bot_start_dist = bot.point.distance_between_points(point)
                else: entry_tot_dist_list.append(entry_outside.distance_between_points(point))
                entry_outside = point
        
        if entry_outside == None or entry_inside == None: return None, None
        
        entry_tot_dist_list.append(entry_outside.distance_between_points(self.crossing_boarder(entry_outside, entry_inside)))
        exit_tot_dist_list.append(exit_outside.distance_between_points(self.crossing_boarder(exit_inside, exit_outside)))
        
        return (bot_start_dist + sum(entry_tot_dist_list), entry_tot_dist_list), (bot_start_dist + sum(exit_tot_dist_list), exit_tot_dist_list)
    
    
    def crossing_boarder(self, outside:Point, inside:Point):
        crossing_vector = Vector(inside.x-outside.x,inside.y-outside.y)
        borders = self.obj.from_local_to_global()
        prev_point = borders[-1]
        crossing_point = None
        distance = math.inf
        
        for point in borders:
            vector = Vector(point.x-prev_point.x, point.y-prev_point.y)
            p = self.obj.crossing_vector(outside, prev_point, crossing_vector, vector)
            if not p == False:
                dist = p.distance_between_points(outside)
                if dist<distance:
                    distance = dist
                    crossing_point = p
            prev_point = point
        return crossing_point
        
    def claim(self, bot:Bot):
        if self.claimed == None: 
            self.claimed = bot.id
            return True
        return False     
    
    def release(self, bot:Bot):
        if self.claimed == bot.id: 
            self.claimed = None
            return True
        return False      




class Intersection:
    def __init__(self, p1:Point, p2:Point, bot_range:float = None, theta:float = 0):
        if bot_range == None: 
            self.range = max(abs(p1.x-p2.x),abs(p1.y-p2.y))
        else:
            self.range = bot_range
        self.p1 = p1
        self.p2 = p2
        self.parts = [Intersection_section(p1,p2,n,theta) for n in range(4)]
        self.bots_in_intersection = []
        
    def update(self):
        self.bots_in_range()

        bot = bots[my_id]
        bot.mti = self.mean_time_to_dist(bot,self.dist_to_entry(bot))
        bot.time_to_exit = self.mean_time_to_dist(bot,self.dist_to_exit(bot)) + 3
        if next_time_step(): self.heart_beat("HB")
        
        self.intersection_ctrl()
    
    def init_new_bot(self, bot:Bot):
        entry_arr, exit_arr, path_len = self.get_dist_lists(bot)
        if len(entry_arr) == 0 or len(exit_arr) == 0: 
            bot.intersection_sections = []
            return
        bot.intersection_entry_dist_list = entry_arr[0][1]
        bot.intersection_exit_dist_list = exit_arr[-1][1]
        bot.intersection_sections = [sublist[3] for sublist in entry_arr]
        bot.start_path_len = path_len
        
    def reset_bot(self, bot:Bot):
        bot.intersection_entry_dist_list = None
        bot.intersection_exit_dist_list = None
        bot.intersection_sections = None
        bot.start_path_len = None
        
    def get_dist_lists(self, bot:Bot):
        path_len = len(bot.path)
        entry_list = []
        exit_list = []
        for part in self.parts:
            entry, exit = part.get_path_dist(bot)
            if entry == None or exit == None: break
            if not entry[1] == None:
                entry_list.append((entry[0], entry[1], part.n))
                entry_list.sort()
            if not exit[1] == None:
                exit_list.append((exit[0], exit[1], part.n))
                entry_list.sort()          
                
        return entry_list, exit_list, path_len
    
    def get_exit_dist(self, bot:Bot):
        qp = (self.p2-self.p1)*0.5
        len(bot.intersection_sections)*max(qp.x,qp.y)
    
    def bots_in_range(self):
        bot_list = []
        for bot in bots.values():
            if self.in_range(bot):
                try:
                    if bot.id == my_id and bot.intersection_sections == None: 
                        self.init_new_bot(bot)
                        if len(bot.intersection_sections) == 0: continue
                    else:
                        continue
                except AttributeError:
                    self.init_new_bot(bot)
                bot_list.append(bot)
        self.bots_in_intersection = bot_list
    
    def in_range(self, bot:Bot):
        pm = self.p2 - self.p1
        return pm.distance_between_points(bot.point)<=self.range

    def claim_parts(self, bot:Bot):
        parts = bot.intersection_sections
        for n in parts:
            if not self.parts[n].claimed in [bot.id, None]: return False
        for n in parts:
            self.parts[n].claim(bot)
        return True
    
    def release_parts(self, bot:Bot):
        parts = bot.intersection_sections
        for n in parts: self.parts[n].release(bot)

    
    def dist_to_entry(self, bot:Bot):
        try:
            index = int(len(bot.start_path_len)-len(bot.path))
            if index > len(bot.intersection_entry_dist_list) : return 0
            dist_list = bot.intersection_entry_dist_list[index:]
            return bot.point.distance_between_points_in_list(dist_list)
        except: 
            return None
    
    def dist_to_exit(self, bot:Bot):
        try:
            index = int(len(bot.start_path_len)-len(bot.path))
            if index > len(bot.intersection_exit_dist_list) : return 0
            dist_list = bot.intersection_exit_dist_list[index:]
            return bot.point.distance_between_points_in_list(dist_list)
        except: 
            return None
    
    def mean_time_to_dist(self, bot:Bot, dist:float):
        if dist == None: return None
        dist = dist/1000
        speed = bot.absolute_speed
        acc = bot.acceleration
        if not acc==0:
            mti = (-speed + math.sqrt(speed**2 + 2*acc*dist))/acc
        elif not speed == 0:
            mti = dist/speed
        else:
            mti = None
        return mti
    
    def bot_in_intersection(self, bot:Bot):
        in_intersection = False
        for part in self.parts:
            in_intersection = in_intersection or part.obj.global_point_in_object(bot.point)
        return in_intersection
        
    def heart_beat(status):
        my_bot = bots[my_id]
        if status == "HB": msg = str(["intersection_1", my_id, "HB", my_bot.mti, my_bot.time_to_exit])
        if status == "ENTER": msg = str(["intersection_1", my_id, "ENTER", my_bot.intersection_sections])
        if status == "EXIT": msg = str(["intersection_1", my_id, "EXIT"])
        for id in bots.keys():
            if id == my_id: continue
            send(ip[id], 2020, msg)
      
    def con_1(self, bot:Bot):
        return type(bot.intersection_sections) is list and len((bot.intersection_sections))>0

    def heart_beat_con(self) -> bool:
        los = False
        for bot in self.bots_in_intersection:
            if bot.id == my_id: continue
            los = los or bot.loss_of_signal()
        return not los
          
    def sort_priority_bot_list(self, priority_list):
        mti_ref = None
        bot_list = []
        time_step = 1
        new_list = []
        for mti,_, bot in priority_list:
            if mti_ref == None : 
                bot_list.append(bot)
                mti_ref = mti
                continue
            if mti-mti_ref < time_step:
                bot_list.append(bot)
            else:
                new_list.extend(sorted(bot_list, key=lambda bot: bot.id))
                bot_list = []
                bot_list.append(bot)
                mti_ref = mti
        new_list.extend(sorted(bot_list, key=lambda bot: bot.id))
        return new_list
            
    def get_priority_bot(self):
        priority_bot_list = []
        for bot in self.bots_in_intersection:
            if not self.con_1(bot): continue
            priority_bot_list.append((bot.mti, bot.id, bot))
            priority_bot_list.sort()
        return self.sort_priority_bot_list(priority_bot_list)
                    
    def bot_communication(self, msg):
        id = msg[0]
        msg_type = msg[1]
        time = rospy.get_time()
        
        if msg_type == "ENTER" : 
            bots[id].intersection_sections = msg[2]
            bots[id].last_update = time
        
        elif msg_type == "EXIT" : 
            
            bots[id].intersection_sections = None
            bots[id].mti = None
            bots[id].time_to_exit = None
            
            self.release_parts(bots[id])
            bots[id].last_update = time
        
        elif msg_type == "HB" : 
            bots[id].mti = msg[2][0]
            bots[id].time_to_exit = msg[2][1]
            bots[id].last_update = time
        
        
    def intersection_ctrl(self):
        if self.heart_beat_con(self):
            
            self.heart_beat("ENTER")
            priority_list = self.get_priority_bot(self)
            
            for bot in priority_list:
                if self.claim_parts(bot):
                    if bot.id == my_id:
                        s.setSpeed(SPEED)
                        if bot.mti == 0 and not self.bot_in_intersection(bot):
                            self.heart_beat("EXIT")
                            self.reset_bot(bot)
                else:
                    if bot.id == my_id:
                        time_to_clear = priority_list[0][1].time_to_exit
                        dist = self.dist_to_entry(bot)
                        new_vel = min(max(dist / time_to_clear, 0), SPEED)
                        s.setSpeed(new_vel)
        else:
            s.setSpeed(0)
    
    
    
    
    
class Receiver:
    def __init__(self):
        rospy.Subscriber('gv_positions', GulliViewPosition, self.positions)
        rospy.Subscriber('status', Status, self.status)
        rospy.Subscriber('path', Polygon, self.path)
        
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
        bots[my_id].add_speed((bots[my_id].right_speed + bots[my_id].left_speed)/2) # Average speed forward, divided by two to match speed on cmdvel

    def path(self, path_msg):
        path = []
        for point in path_msg.points:
            path.append(Point(point.x,point.y))
        bots[my_id].path = path

        

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
        cooperative_controller[data_list[0]].bot_communication(data_list[1:])

        

def wait_for_path(bot:Bot):
    if len(bot.path) > 0: return
    s.setSpeed(0)
    while True: 
        if len(bot.path) > 0: return


cooperative_controller = {"intersection 1":Intersection(Point(0,0), Point(0,0), 0)}

def run():
    for controller in cooperative_controller.values():
        controller.update()


if __name__ == '__main__': 
    run_flag = threading.Event()
    run_flag.set()
    t = None
    try:
    
        d = Data()
        s = Speed()
        
        my_id = int(sys.argv[1])
        operation = sys.argv[2]
        bots = {my_id:Bot(my_id)}
        ip = {4:'192.168.50.102',5:'192.168.50.103'}
        
        node = Receiver()
        
        t = threading.Thread(target=listen, args=(run_flag, ip[my_id], 2020, d))
        t.start()
        
        wait_for_path(bots[my_id])
        
        while not rospy.is_shutdown():
            run()

                
    except KeyboardInterrupt:
        run_flag.clear()
        if not t is None: t.join()
    finally:
        run_flag.clear()
        if not t is None: t.join()
        

            

