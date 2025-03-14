#!/bin/env python3

import rospy
from gv_client.msg import GulliViewPosition, LaptopSpeed
from roswifibot.msg import Status
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon
from go_to_goal import SPEED
import sys
import socket
import threading
import math
import os
import datetime
import time

rospy.init_node('cooperative_maneuver_control', anonymous=True)
rospy.loginfo("Starting cooperative_maneuver_control node")

time_step = 1
last_time_step = rospy.get_time()
 
def init_log():
    directory = os.path.dirname(os.path.abspath(__file__))
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"log_{timestamp}.txt"
    filepath = os.path.join(directory, filename)
    i = 1
    while os.path.exists(filepath):
        filename = f"log_{timestamp}_{i}.txt"
        filepath = os.path.join(directory, filename)
        i += 1
    with open(filepath, "w") as file:
        file.write("Data\n")
    return filepath
        
def log(filepath, name:str, data):
    time = datetime.datetime.now().strftime("%M-%S-%f")
    data = str(data)
    #print(f"Time: {time}, {name}: {data}")
    with open(filepath, "a") as file:
        file.write(f"Time: {time}, {name}: {data}\n")

def next_time_step() -> bool:
    global last_time_step
    if rospy.get_time() > last_time_step + time_step:
        last_time_step = float(rospy.get_time())
        return True
    return False


class ThreadSafeDict:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {}

    def __getitem__(self, key):
        with self.lock:
            if key in self.data:
                return self.data[key]  
            return None

    def __setitem__(self, key, value):
        with self.lock:
            self.data[key] = value

    def __delitem__(self, key):
        with self.lock:
            del self.data[key]

    def __contains__(self, key):
        with self.lock:
            return key in self.data

    def keys(self):
        with self.lock:
            return list(self.data.keys())

    def values(self):
        with self.lock:
            return list(self.data.values())


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
        if local_point == None: return None
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
        self_collision_point = None
        
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
                if distance < closest_distance: 
                    closest_distance = distance
                    self_collision_point = obj_collision_point
        
        return self.point_from_local_to_global(front_point).distance_between_points(self.point_from_local_to_global(self_collision_point))

    
    def crossing_vector(self, self_point:Point, obj_point:Point, self_vel:Vector, obj_vel:Vector):
        try:
            if not obj_vel.x == 0:
                self_dist = (obj_point.y-self_point.y+obj_vel.y*((self_point.x-obj_point.x)/obj_vel.x))/(self_vel.y-(obj_vel.y*self_vel.x)/obj_vel.x)
                return (self_vel*self_dist).to_point() + self_point 
            elif not self_vel.x == 0:
                self_dist = (self_point.y-obj_point.y+self_vel.y*((obj_point.x-self_point.x)/self_vel.x))/(obj_vel.y-(self_vel.y*obj_vel.x)/self_vel.x)
                return (obj_vel*self_dist).to_point() + obj_point 
        except ZeroDivisionError:
            return None
     
    def moving_collision_course(self, obj):
        self_p1, self_p2, front_point = self.get_outer_points()
        obj_p1, obj_p2, _ = obj.get_outer_points()
        
        self_p1 = self.point_from_local_to_global(self_p1)
        self_p2 = self.point_from_local_to_global(self_p2)
        front_point = self.point_from_local_to_global(front_point)
        obj_p1 = obj.point_from_local_to_global(obj_p1)
        obj_p2 = obj.point_from_local_to_global(obj_p2)
        
        self_vel = self.get_global_velocity_vector()
        obj_vel = obj.get_global_velocity_vector()
    
        crossing_point_1 = self.crossing_vector(self_p1, obj_p2, self_vel, obj_vel)
        crossing_point_2 = self.crossing_vector(self_p2, obj_p1, self_vel, obj_vel)

        return front_point.distance_between_points(crossing_point_1), front_point.distance_between_points(crossing_point_2)
        
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
        self.time_to_bot_collision = {}
        
    def __str__(self):
        return f"id: {self.id}, pos; {self.point}, speed: {self.absolute_speed}"
    
    def __getattribute__(self, name):
        try: return super().__getattribute__(name)
        except AttributeError: return None
    
    def update_pos(self, point:Point, theta:float):
        self.point = point
        self.theta = theta
        self.obj.update(self.point, self.theta)
        
    def add_speed(self, speed:float):
        time = rospy.get_time()
        if not self.last_speed_update is None and not self.last_speed_update is None and time-self.last_speed_update > 0:
            self.acceleration = (speed-self.absolute_speed)/(time-self.last_speed_update) # [m/s^2]
        self.last_speed_update = time 
        self.absolute_speed = speed
    
    def loss_of_signal(self):
        return rospy.get_time() - self.last_update > time_step + 1
    
    def update_collision_point(self, bot):
        dist = self.obj.collision_course(bot.obj)
        dist1, dist2 = self.obj.moving_collision_course(bot.obj)
        collision_list = [dist, dist1, dist2]
        collision_list = sorted(collision_list, key=lambda x: (x is None, x))
        self.time_to_bot_collision[bot.id] = (collision_list[0]/1000)/self.absolute_speed # m / (m/s) = s
        bot.time_to_bot_collision[self.id] = (collision_list[0]/1000)/bot.absolute_speed


        
    
class Intersection:
    class Intersection_section:
        def __init__(self, p1:Point, p2:Point, n:int, theta:float):
            dx = abs(p2.x-p1.x)
            dy = abs(p2.y-p1.y)
            self.n = n
            self.center_point = p1 + Point(dx/4 + int(n%2)*dx/2, dy/4 + int(math.floor(n/2))*dy/2)
            self.obj = Object(self.center_point, theta, [Point(-dx/4,-dy/4),Point(-dx/4,dy/4),Point(dx/4,dy/4),Point(dx/4,-dy/4)])
            self.claimed = None

        def __repr__(self):
            return f"{self.n} : {self.claimed}"
        
        def __str__(self):
            return f"part {self.n} is claimed by {self.claimed} \n"

        def get_path_dist(self, bot:Bot, path:list):
            entry_outside = None
            entry_inside = None
            entry_tot_dist_list = []
            exit_outside = None
            exit_inside = None
            exit_tot_dist_list = []
            bot_start_dist = 0
            
            for i in range(len(path)):
                point = path[i]
                if self.obj.global_point_in_object(point): 
                    entry_inside = point
                    exit_tot_dist_list = entry_tot_dist_list.copy()

                    for point in path[i:]:
                        if not self.obj.global_point_in_object(point): 
                            exit_outside = point
                            break
                        else: 
                            if exit_inside == None: exit_tot_dist_list.append(entry_outside.distance_between_points(entry_inside))
                            else: exit_tot_dist_list.append(exit_inside.distance_between_points(point))
                            exit_inside = point
                    break
                else: 
                    if entry_outside == None: bot_start_dist = bot.point.distance_between_points(point)
                    else: entry_tot_dist_list.append(entry_outside.distance_between_points(point))
                    entry_outside = point
            
            if entry_outside == None or entry_inside == None: return None, None
            
            entry_point = self.crossing_boarder(entry_outside, entry_inside)
            exit_point = self.crossing_boarder(exit_outside, exit_inside)

            entry_tot_dist_list.append(entry_outside.distance_between_points(entry_point))
            exit_tot_dist_list.append(exit_inside.distance_between_points(exit_point))

            return (bot_start_dist + sum(entry_tot_dist_list), entry_tot_dist_list, entry_point), (bot_start_dist + sum(exit_tot_dist_list), exit_tot_dist_list, exit_point)
        
        def crossing_boarder(self, outside:Point, inside:Point):
            crossing_vector = Vector(inside.x-outside.x,inside.y-outside.y)
            borders = self.obj.from_local_to_global()
            prev_point = borders[-1]
            crossing_point = None
            distance = math.inf
            
            for point in borders:
                vector = Vector(point.x-prev_point.x, point.y-prev_point.y)
                p = self.obj.crossing_vector(outside, prev_point, crossing_vector, vector)
                if not p == None:
                    dist = p.distance_between_points(outside)
                    if dist<distance:
                        distance = dist
                        crossing_point = p
                prev_point = point
            return crossing_point
            
        def claim(self, bot:Bot):
            if self.claimed == None: 
                log(file, f"Intersection part {self.n}", f"claimed by {bot.id}")
                self.claimed = bot.id
                return True
            return False     
        
        def release(self, bot:Bot):
            if self.claimed == bot.id: 
                log(file, f"Intersection part {self.n}", f"released by {bot.id}")
                self.claimed = None
                return True
            return False      

    class Bot_param:
        def __init__(self, outer, bot:Bot):
            self.outer = outer
            self.bot = bot
            self.lock = threading.Lock()
            log(file, "New bot params with id", bot.id)
            
        def init_my_bot(self):
            log(file, "Status", "init inter params")
            path = self.bot.path.copy()
            path_len = len(path)
            entry_list = []
            exit_list = []
    
            for part in self.outer.parts:
                entry, exit = part.get_path_dist(self.bot, path)
                if entry == None or exit == None: continue
                if not entry[1] == None:
                    entry_list.append((len(entry[1]), entry[1], entry[2], part.n))
                
                if not exit[1] == None:
                    exit_list.append((len(exit[1]), exit[1], exit[2], part.n))
                
            entry_list.sort()
            exit_list.sort()          
                    
            if len(entry_list) == 0 or len(exit_list) == 0: 
                self.intersection_sections = []
                log(file, "Status", "no path through inter")
                return
            self.intersection_entry_dist_list = entry_list[0][1]
            self.intersection_entry_point = entry_list[0][2]
            self.intersection_exit_dist_list = exit_list[-1][1]
            self.intersection_exit_point = exit_list[-1][2]
            self.intersection_sections = [sublist[3] for sublist in entry_list]
            self.start_path_len = path_len
            log(file, "Path through inter", f"inter parts: {self.intersection_sections}, entry: distlist: {self.intersection_entry_dist_list}, point: {self.intersection_entry_point}; exit: distlist: {self.intersection_exit_dist_list}, point: {self.intersection_exit_point}")
        
        
        def dist_to_border(self, dist_list:list, last_point:Point, last_con:bool):
            try:
                index = int(self.start_path_len-len(self.bot.path))
                if index > len(dist_list) : 
                    return 0
                dist_list = dist_list[index:]
                bot_to_point_dist = self.bot.point.distance_between_points(self.bot.path[0])
                if len(dist_list) <= 1: 
                    if last_con: 
                        return 0
                    return self.bot.point.distance_between_points(last_point)
                return sum(dist_list) + bot_to_point_dist
            except: 
                return None
        
        def dist_to_entry(self):
            last_con = self.outer.bot_in_intersection(self.bot)
            return self.dist_to_border(self.intersection_entry_dist_list, self.intersection_entry_point, last_con)
        
        def dist_to_exit(self):
            dti = self.bot.point.distance_between_points(self.intersection_entry_point)
            dbee = self.intersection_entry_point.distance_between_points(self.intersection_exit_point)
            last_con = dti > dbee
            return self.dist_to_border(self.intersection_exit_dist_list, self.intersection_exit_point, last_con)
         
        def mean_time_to_dist(self, bot:Bot, dist:float):
            if dist == None: return math.inf
            dist = dist/1000
            speed = bot.absolute_speed
            acc = bot.acceleration
            # if not acc==0:
            #     
            #     mti = (-speed + math.sqrt(speed**2 + 2*acc*dist))/acc
            if not speed == 0:
                mti = dist/speed
            else:
                mti = math.inf
            return mti
        
        def __getattribute__(self, name):
            try: return super().__getattribute__(name)
            except AttributeError: return None
            
        def update_times(self):
            self.mti = self.mean_time_to_dist(self.bot, self.dist_to_entry())
            self.time_to_exit = self.mean_time_to_dist(self.bot, self.dist_to_exit() + 0.20) # snabb fix
            
    
    def __init__(self, name:str, p1:Point, p2:Point, theta:float = 0, bot_range:float = None):
        self.name = name
        self.range = bot_range if not bot_range == None else max(abs(p1.x-p2.x),abs(p1.y-p2.y))*2#1.1
        self.p1 = p1
        self.p2 = p2
        self.parts = [self.Intersection_section(p1,p2,n,theta) for n in range(4)]
        self.bot_params = ThreadSafeDict()
        
    
    def update(self):
        self.bots_in_range()
        if self.conecondition_one(): self.bot_params[my_id].update_times()
        self.heart_beat("HB")
        self.intersection_ctrl()
    
    def in_range(self, bot:Bot, max_range:float = None):
        if max_range == None: max_range = self.range
        center_point = (self.p2 - self.p1)*0.5 + self.p1
        return center_point.distance_between_points(bot.point) <= max_range
    
    def bots_in_range(self):
        for bot in bots.values():
            if self.in_range(bot) and not bot.id in self.bot_params:
                self.bot_params[bot.id] = self.Bot_param(self, bot)
                if bot.id == my_id: 
                    self.bot_params[bot.id].init_my_bot()
                 
    def remove_bot(self, bot:Bot):
        with self.bot_params[bot.id].lock:
            log(file, f"Removed bot with id", bot.id)
            self.release_parts(bot)
            del self.bot_params[bot.id]

    def claim_parts(self, bot:Bot):
        if not bot.id in self.bot_params: return
        with self.bot_params[bot.id].lock:
            if not bot.id in self.bot_params: return
            log(file, f"Claimes", f"bot {bot.id} want {self.bot_params[bot.id].intersection_sections}")
            parts = self.bot_params[bot.id].intersection_sections
            if parts == None: return False
            for n in parts:
                if not self.parts[n].claimed in [bot.id, None]: return False
            log(file, f"Claimes", f"bot {bot.id} got {self.bot_params[bot.id].intersection_sections}")
            for n in parts:
                self.parts[n].claim(bot)
            return True
    
    def release_parts(self, bot:Bot):
        if not bot.id in self.bot_params: return
        parts = self.bot_params[bot.id].intersection_sections
        for n in parts: self.parts[n].release(bot)

    def bot_in_intersection(self, bot:Bot):
        in_intersection = False
        for part in self.parts:
            in_intersection = in_intersection or part.obj.global_point_in_object(bot.point)
        return in_intersection
        
    def heart_beat(self, status:str, receiver_id:int = None):
        if not my_id in self.bot_params: return
        mti = self.bot_params[my_id].mti
        mte = self.bot_params[my_id].time_to_exit
        intsec = self.bot_params[my_id].intersection_sections
        if intsec == []: intsec = None
        
        if status == "HB": msg = str([self.name,my_id,"HB",intsec,mti,mte])
        if status == "ENTER": msg = str([self.name,my_id,"ENTER",intsec,mti,mte])
        if status == "EXIT": msg = str([self.name,my_id,"EXIT"])
        
        if receiver_id == None:
            for id in self.bot_params.keys():
                if not id in ip.keys():continue
                #print(id)
                if id == my_id: continue
                udp_client.send(ip[id], 2020, msg)
        elif receiver_id in ip.keys():
            if not receiver_id in ip.keys():return
            udp_client.send(ip[receiver_id], 2020, msg)
      
    def conecondition_one(self):
        if not my_id in self.bot_params: return False
        intsec = self.bot_params[my_id].intersection_sections
        return type(intsec) is list and len(intsec)>0

    def heart_beat_con(self) -> bool:
        los = False
        for param in self.bot_params.values():
            bot = param.bot
            if bot.id == my_id: continue
            los = los or bot.loss_of_signal()
        return not los 
          
    def sort_priority_bot_list(self, priority_list:list):
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
        for param in self.bot_params.values():
            bot = param.bot
            mti = param.mti #if not param.mti == None else math.inf
            if mti == None: continue
            priority_bot_list.append((mti, bot.id, bot))
            priority_bot_list.sort()
        return self.sort_priority_bot_list(priority_bot_list)
                    
    def bot_communication(self, msg:list):
        id = msg[0]
        msg_type = msg[1]
        time = rospy.get_time()

        if not id in self.bot_params: return
        params = self.bot_params[id]
        
        if msg_type == "ENTER" : 
            params.intersection_sections = msg[2]
            params.mti = msg[3]
            params.time_to_exit = msg[4]
            bots[id].last_update = time
        
        elif msg_type == "EXIT" : 
            self.remove_bot(bots[id])
            bots[id].last_update = time

        elif msg_type == "HB" : 
            params.intersection_sections = msg[2]
            params.mti = msg[3]
            params.time_to_exit = msg[4]
            bots[id].last_update = time
            

        
        
    def intersection_ctrl(self):
        my_bot = bots[my_id]
        if self.in_range(my_bot, self.range*0.8) and self.conecondition_one(): 
            log(file, "Status", "bot in entry range")
            if self.heart_beat_con():
                
                self.heart_beat("ENTER")
                
                if self.heart_beat_con():
                    priority_list = self.get_priority_bot()
                    log(file, "Priority list", priority_list)
                    
                    for bot_index in range(len(priority_list)):
                        bot = priority_list[bot_index]
                        if self.claim_parts(bot):
                            if bot.id == my_id:
                                log(file, "Status", "Entering")
                                topic_handler.setSpeed(SPEED)
                                if self.bot_params[my_id].dist_to_exit() == 0:
                                    log(file, "Status", "Exiting")
                                    self.heart_beat("EXIT")
                                    self.remove_bot(my_bot)
                                    return
                        else:
                            if bot.id == my_id:
                                log(file, "Status", "Slowing down")
                                params = self.bot_params[priority_list[bot_index-1].id]
                                if params == None: return
                                time_to_clear = params.time_to_exit
                                if time_to_clear == None: return
                                dist = self.bot_params[my_id].dist_to_entry()/1000 - 0.16 ## snabb fix
                                new_vel = min(max(dist / time_to_clear, 0), SPEED)
                                log(file, "Slowing down: ", f"dist: {(dist+0.16)*1000}, new_vel: {new_vel}")
                                if new_vel <= 0.015: new_vel = 0
                                topic_handler.setSpeed(new_vel)
                    return
            topic_handler.setSpeed(0)
            log(file, "Status", "stoping, loss of signal with bots")




class Ros_topic_handler:
    def __init__(self):
        self.publisher = rospy.Publisher('gv_laptop', LaptopSpeed, queue_size=10)
        rospy.Subscriber('gv_positions', GulliViewPosition, self.positions)
        rospy.Subscriber('status', Status, self.status)
        rospy.Subscriber('path', Polygon, self.path)
        
    def positions(self, position_msg:GulliViewPosition):
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

    def status(self, status_msg:Status):
        bots[my_id].left_speed = status_msg.speed_front_left/2
        bots[my_id].right_speed = status_msg.speed_front_right/2
        bots[my_id].add_speed((bots[my_id].right_speed + bots[my_id].left_speed)/2) # Average speed forward, divided by two to match speed on cmdvel

    def path(self, path_msg:Polygon):
        path = []
        for point in path_msg.points:
            path.append(Point(point.x,point.y))
        bots[my_id].path = path
    
    def setSpeed(self, speed:float):
        log(file, "Setting speed", f"{speed} m/s")
        header = Header()
        header.stamp = rospy.Time.now()
        msg = LaptopSpeed(header=header, tag_id=my_id, speed=speed, restart=False)
        self.publisher.publish(msg)
    
    
class UDP_client:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def send(self, adress:str, port:int, msg:str):
        log(file, "Sending", f"{msg} to {adress}")
        self.sock.sendto(msg.encode(), (adress, port))
    
    def close(self):
        self.sock.close()


class UDP_server(threading.Thread):
    def __init__(self, adress:str, port:int, timeout:float = 2):
        super().__init__()
        self.adress = adress
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((adress, port))
        self.running = True
        self.timeout = timeout
        
    def run(self):
        self.sock.settimeout(self.timeout)
        while self.running:
            try:
                data, sender_adress = self.sock.recvfrom(4096)
                data = data.decode()
                self.handler(data)
            except socket.timeout:
                continue
            except OSError:
                break 
           
    def stop(self):
        self.running = False
        self.sock.close()
        
    def handler(self, data:str):
        try: data_list = self.to_list(data)
        except: return
        log(file, "Data from udp", data_list)
        cooperative_controller[data_list[0]].bot_communication(data_list[1:])  
        
    def isfloat(self, num):
        try:
            float(num)
            return True
        except ValueError:
            return False

    def to_list(self, string:str):
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
            elif element == "None": element = None
            else: element = element[1:-1]
            arr[i] = element
        return arr
    



cooperative_controller = {"intersection 1":Intersection("intersection 1",Point(2845, 2782), Point(4489, 4605), 0)}

def run():
    for controller in cooperative_controller.values():
        controller.update()

def wait_for_path(bot:Bot):
    try:
        log(file, "Status", "waiting for path")
        if len(bot.path) > 0: return
        topic_handler.setSpeed(0)
        while not rospy.is_shutdown(): 
            if len(bot.path) > 0: 
                log(file, "Status", "Path received")
                topic_handler.setSpeed(SPEED)
                return
            time.sleep(0.1) 
    except OSError:
        return 


if __name__ == '__main__': 
    try:
        my_id = int(sys.argv[1])
        bots = ThreadSafeDict()
        ip = ThreadSafeDict()
        
        bots[my_id]=Bot(my_id)
        ip[4] = '192.168.50.102'
        ip[5] = '192.168.50.103'
        
        file = init_log()
        topic_handler = Ros_topic_handler()
        udp_client = UDP_client()
        udo_server = UDP_server(ip[my_id], 2020)
        udo_server.start()
        log(file, "Status", "Starting udp-server")

        wait_for_path(bots[my_id])
        
        while not rospy.is_shutdown():
            run()

    except KeyboardInterrupt:
        udo_server.stop()
        udp_client.close()
    finally:
        udo_server.stop()
        udp_client.close()
        

           