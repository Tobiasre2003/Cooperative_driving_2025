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
    
    def ang_to_point(self, x, y):
        return math.atan2(x-self.x, y-self.y) % 2*math.pi
    
    
class Line:
    def __init__(self, p1:Point, p2:Point):
        self.p1 = p1
        self.p2 = p2

    def get_lin_func(self):
        if self.p1.x != self.p2.x:
            k = (self.p2.y-self.p1.y)/(self.p2.x-self.p1.x)
            m = self.p2.y - k*self.p1.x
        else: return (self.p1.x)
        return (k,m)
    
    def crossing(self, line): 
        l1 = self.get_lin_func()
        l2 = line.get_lin_func()
        
        if len(l1)==1 and len(l2)==1: 
            if self.p1.x == line.p1.x: return (self.p1.x, range(-math.inf,math.inf))
            return (None, None)
        elif len(l1)==1: 
            k = l2[0]
            m = l2[1]
            return (self.p1.x, self.p1.x*k+m)
        elif len(l2)==1:
            k = l1[0]
            m = l1[1]
            return (line.p1.x, line.p1.x*k+m)
        
        k1 = l1[0]
        m1 = l1[1]
        k2 = l2[0]
        m2 = l2[1]
        
        if k1 == 0 and k2 == 0: 
            if self.p1.y == line.p1.y: return (range(-math.inf,math.inf), self.p1.y)
            return (None, None)

        x = (m2 - m1)/(k1 - k2)
        y = m1 + k1 * x 
        return (x,y)

    
        
        
        
    
        
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
        
        
    def add_speed(self, speed:float):
        time = rospy.get_time()
        if not self.last_speed_update is None:
            self.acceleration = (speed-self.absolute_speed)/(time-self.last_speed_update) # [m/s^2]
        self.last_speed_update = time 
        self.absolute_speed = speed
                
    def __repr__(self):
        return f'(Id: {self.id}, Left speed: {self.left_speed}, Right speed: {self.right_speed}, Absolute speed: {self.absolute_speed}, Point: {self.point})'

class Area:
    def __init__(self, lines:list[Line]): 
        self.lines = lines
        
    def calc_angs(self):None
        
    def in_area(self, x:float, y:float):
        None

    
    def heading_towards_area(self, bot:Bot):
        point = bot.point
        angles = []
        
        for line in self.lines:
            angles.append(point.ang_to_point(line.p1.x, line.p1.y))
            angles.append(point.ang_to_point(line.p2.x, line.p2.y))
        
        max_angle = max(angles)
        min_angle = min(angles)
        
        theta = bot.theta % 2*math.pi

        if max_angle - min_angle > math.pi:
            theta = (theta + 2*math.pi) % 2*math.pi
            min_angle = (min_angle + 2*math.pi) % 2*math.pi
            max_angle = (max_angle + 2*math.pi) % 2*math.pi
            
        return min_angle <= theta <= max_angle, angles



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
            bots[id].point = self.p
            bots[id].theta = theta
        else: 
            bots[id] = Bot(id)
            bots[id].point = self.p
            bots[id].theta = theta

   
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
        

            

