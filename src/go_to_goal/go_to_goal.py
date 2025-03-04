#!/bin/env python3

# Here we will experiment with go to goal

# 102

from asyncore import write
from curses import KEY_PPAGE
from math import atan2, sin, cos, pi
from operator import length_hint
import time
import csv
import rospy
import sys
import random
from geometry_msgs.msg import Twist, Point, Polygon, Point32
from gv_client.msg import GulliViewPosition, LaptopSpeed


from roswifibot.msg import IR



SPEED = 0.3
ERROR_RANGE = 300 

# Origin point to return to base
ORIGIN_POINT = [Point(2000, 9000, 0)]

# (Old) Merging on-ramp path, right lane
# RAMP_PATH = [Point(3702, 8547, 0), Point(2054, 7150, 0), Point(1711, 6332, 0), Point(1640, 5200, 0), Point(1180, 4260, 0), Point(1033, 2168, 0)]

# Merging on-ramp path, right lane
#RAMP_PATH = [Point(3702, 8547, 0), Point(2054, 7150, 0), Point(1711, 6332, 0), Point(1640, 5900, 0), Point(1180, 4260, 0), Point(1033, 2168, 0)]

# (Old) Merging on-ramp path, right lane
# RAMP_PATH = [Point(3702, 8547, 0), Point(2054, 7150, 0), Point(1711, 6332, 0), Point(1640, 5200, 0), Point(1180, 4260, 0), Point(1033, 2168, 0)]

# Merging on-ramp path, right lane
#RAMP_PATH = [Point(3702, 8547, 0), Point(2054, 7150, 0), Point(1711, 6332, 0), Point(1640, 5900, 0), Point(1180, 4260, 0), Point(1033, 2168, 0)]

# Less points
RAMP_PATH = [Point(3702, 8547, 0), Point(3702, 8547, 0), Point(2054, 7150, 0), Point(1033, 3000, 0)]
#Point(2502, 7850, 0),         Point(3320, 3200, 0),
LOOP_RAMP_PATH = [Point(3320, 2800, 0), Point(3320, 5600, 0), Point(3320, 6000, 0), Point(3702, 6847, 0), Point(3452, 8247, 0), Point(3702, 8247, 0) ]
#LOOP_RAMP_PATH = [Point(3320, 2800, 0), Point(3320, 3500, 0), Point(3320, 4600, 0), Point(3320, 5600, 0), Point(3320, 6600, 0), Point(3702, 8547, 0), Point(2502, 7850, 0), Point(3702, 8547, 0) ]

# Merging main road path, left lane
# MAIN_PATH = [Point(480, 8400, 0), Point(480, 7400, 0), Point(500, 6400, 0), Point(480, 5400, 0), Point(450, 4400, 0), Point(500, 3400, 0), Point(500, 2400, 0), Point(480, 2100, 0)]

# Merging main road path, right lane
MAIN_PATH = [Point(900, 8400, 0), Point(900, 7400, 0), Point(900, 6400, 0), Point(900, 5400, 0), Point(900, 4400, 0), Point(900, 3400, 0), Point(900, 3000, 0)]

LOOP_MAIN_PATH = [Point(3320, 2800, 0),  Point(3320, 5600, 0), Point(3320, 6000, 0), Point(2630, 8100, 0), Point(870, 8361, 0), Point(830, 8361, 0) ]

INTERSECTION_PATH = [Point(1200, 8200, 0), Point(2200, 6800, 0), Point(3200, 5400, 0),
                     Point(2200, 4600, 0), Point(1200, 5400, 0), Point(2200, 6800, 0),
                     Point(3200, 8200, 0), Point(2200, 9000, 0)]


INTERSECTION_PATH_1 = [Point(3968,7741,0), Point(3968, 4200,0), Point(3628, 3714,0), Point(1852, 3500,0)]
INTERSECTION_PATH_2 = [Point(3487,805,0), Point(3590, 2637,0), Point(3578, 3714,0), Point(3471, 6065,0)]


PATH = []
PATHS = {
        'origin' : ORIGIN_POINT,
        'ramp' : RAMP_PATH,
        'main' : MAIN_PATH,
        'intersection' : INTERSECTION_PATH,
	'intersection_1': INTERSECTION_PATH_1,
	'intersection_2': INTERSECTION_PATH_2
        }

LOOP_PATHS = {
        'origin' : ORIGIN_POINT,
        'ramp' : LOOP_RAMP_PATH,
        'main' : LOOP_MAIN_PATH,
        'intersection' : INTERSECTION_PATH,
	'intersection_1': INTERSECTION_PATH_1,
	'intersection_2': INTERSECTION_PATH_2	
        }

KP = 1
KI = 0
KD = 0

IGNORE_TIMER = 10000
IR_WINDUP = 10



WINDUP_GUARD = 100.0


class GoToGoalNode:

    Left_front_ir_data = 0
    Right_front_ir_data = 0
    Left_back_ir_data = 0
    Right_back_ir_data = 0

    Left_front_ir_previous_value = 0
    Right_front_ir_previous_value = 0

    Left_back_stale_counter = 0
    Right_back_stale_counter = 0



    isSleeping = False
    sleepTimer = 0
    IRwindup = 0
    IRstartupTimer = 0


    loopi = 0

    waitBeforeLooping = False

    def __init__(self):
        rospy.init_node('reachGoal', anonymous=True)
        rospy.loginfo("Starting reachGoal node NEW VERSION")

        rospy.Subscriber('gv_positions', GulliViewPosition, self._position_cb)
        rospy.Subscriber('gv_laptop', LaptopSpeed, self._speed_cb)

        #rospy.Subscriber("/IR", IR, self._callback)

        

        # Keeping track of ir sensor data
        self.Left_front_ir_data = 0
        self.Right_front_ir_data = 0
        self.Left_back_ir_data = 0
        self.Right_back_ir_data = 0

        self.Left_front_ir_previous_value = 0
        self.Right_front_ir_previous_value = 0

        self.Left_back_stale_counter = 0
        self.Right_back_stale_counter = 0

        self.sleepTimer = 0
        self.IRwindup = IR_WINDUP
        self.IRstartupTimer = 0


        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Polygon, queue_size=3)

        self.speed = SPEED
        self.omega = 0
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.pid = PID(self.kp, self.ki, self.kd, WINDUP_GUARD)

        self.i = 0  # index for where in path we are
        self.loopi = 0
        self.waitBeforeLooping = False

        self.dest = PATH[self.i]
        #self.dest = DEST2
        
        self.end_angles = []

        self.rate = rospy.Rate(10)

    # Update speed every time speed msg is recieved
    def _speed_cb(self, speed_msg):
        if speed_msg.restart == True:
            self.waitBeforeLooping = False

        safe_speed = 1.0
        if(abs(speed_msg.speed) > safe_speed):
            print()
            print('_'*32)
            print(f'[!] Safety Cut-off: speed from rostopic recieved: {speed_msg.speed}')
            print('_'*32)
            print()
            if (speed_msg.speed > 0):
                self.speed = safe_speed
        else:
            # Set speed from the msg
            self.speed = speed_msg.speed

    # Update omega every time a GulliViewPosition msg is received
    
    def pub_path(self, PATH):
        path = Polygon()
        path.points = []
        for point in PATH:
            path.points.append(Point32(x=point.x, y=point.y, z=0.0))
        self.path_publisher.publish(path)

    def _position_cb(self, position_msg):
        print("POS IN") # DEBUG
        self.x = position_msg.x
        self.y = position_msg.y
        self.theta = position_msg.theta

        # Change point if within range, else update omega
        if ((self.dest.x + ERROR_RANGE) > self.x > (self.dest.x - ERROR_RANGE)
                and (self.dest.y + ERROR_RANGE) > self.y > (self.dest.y - ERROR_RANGE)):
            self.i += 1
            self.pub_path(PATH[self.i:])
            print("REACHED POINT", self.i)
            if self.i < len(PATH):
                self.dest = PATH[self.i]
            if self.i >= len(PATH):
                print("ENTERED LOOP PATH", self.i)
                self.dest = LOOP_PATH[self.loopi]
                self.loopi += 1

            
        else:
             #max_omega  = self.pid.update(pi, False)
            #Check if the wifibot has already passed a point and in that case go to the other
            # avoids it turning in a circle if sligthly off starting point
            print("pos Y: ", self.y, ", Dest Y: ", self.dest.y, "pos X: ", self.x, ", Dest X: ", self.dest.x,  "i: ", self.i, "len: ", len(PATH))
            #if(self.y < self.dest.y and self.i < len(PATH) and self.loopi == 0):    
            #    self.i += 1
            #    self.dest = PATH[self.i]
            #    print("Inside new function")
            #else:

                #max_omega  = self.pid.update(pi, False)
            true_omega = self.pid.update(self.error(), True)
            
                # Rotation controls are within +-1 so must divide
            self.omega = true_omega#/max_omega
                #print("OMEGA:", self.omega) # DEBUG

        print("POS OUT, SPEED: ", self.speed) # DEBUG


    def _callback(self, data):
        # print("aaaa")
        # print(test) 
        # rospy.loginfo(rospy.get_caller_id() + " IR : %s  --  %s", data.IR_front_left, data.IR_front_right)

        #print(self.Left_front_ir_data) 

#region !------------------------ IR LOGIC  ------------------------!




        '''
        if (self.Left_front_ir_previous_value == data.IR_front_left):
            # rospy.loginfo(rospy.get_caller_id() + " !!! IR SAME VALUE LEFT !!! : %s  --  %s -- %s",  self.Left_back_stale_counter, self.Left_front_ir_previous_value, data.IR_front_left)
            self.Left_back_stale_counter += 1
        else :
            self.Left_back_stale_counter = 0
        
        if (self.Right_front_ir_previous_value == data.IR_front_right):
            # rospy.loginfo(rospy.get_caller_id() + " !!! IR SAME VALUE RIGHT !!! : %s  --  %s --  %s", self.Right_back_stale_counter, self.Right_front_ir_previous_value, data.IR_front_right)
            self.Right_back_stale_counter += 1
        else :
             self.Right_back_stale_counter = 0
        '''
        #update prev value
        self.Left_front_ir_previous_value = data.IR_front_left
        self.Right_front_ir_previous_value = data.IR_front_right

        if (data.IR_front_left <= 40 or (self.speed >= 0.50 and data.IR_front_left <= 80)):
            #print("Test")
            #print(self.Left_front_ir_data) 
            if(self.Left_front_ir_data < 1):
                self.Left_front_ir_data += 1
            else:
                self.Left_front_ir_data = 0
                

        if (data.IR_front_right <= 40 or (self.speed >= 0.50 and data.IR_front_left <= 80)):
            if(self.Right_front_ir_data < 1):
                self.Right_front_ir_data += 1
            else:
                self.Right_front_ir_data = 0

        if ((self.Right_front_ir_data >= 1 or self.Left_front_ir_data >= 1) and (self.Right_back_stale_counter < 3 or self.Left_back_stale_counter < 3)):
            self.Right_front_ir_data = 0
            self.Left_front_ir_data = 0
            #self.speed = 0
            if ( self.IRwindup <= 0 ):
                # self.speed = 0
                node.sleepTimer = IGNORE_TIMER
                rospy.loginfo(rospy.get_caller_id() + " -- !!! IR FAILSAFE SLEEP !!!--  LEFT: " + str(data.IR_front_left)  + "  RIGHT: " + str(data.IR_front_right))
            else:
                self.IRwindup -= 1
                if ( self.IRwindup <= 0 ):
                    rospy.loginfo(rospy.get_caller_id() + " -- !!! IR ACTIVE !!!-- ")
   
#endregion ------------------------ IR LOGIC  ------------------------



# - move to start point, slow down before reaching it, stop when reached, wait for other robot, begin test again

    def move(self, speed, omega):
        twist = Twist()
        twist.linear.x = float(speed)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = float(omega)
        self.publisher.publish(twist)

    # Returns error (which is used in PID.update to calculate omega)
    def error(self):
        end_angle = atan2(self.dest.y-self.y, self.dest.x-self.x)
        self.end_angles.append(end_angle)
        error = end_angle - self.theta

        # Correct error so it stays within +-pi, and invert to fit with controls
        error = -atan2(sin(error), cos(error))

        # DEBUG
        print("X:", self.x, "Y:", self.y, "ERROR:", error) # , f"(end_angle: {end_angle}, theta: {self.theta})")

        return error

    def shutdown(self):
        print(self.end_angles[0])
        # f = open('~/control_vehicles2/test/test_data', 'w')
        # writer = csv.writer(f)
        # writer.writerow(self.end_angles)
        #f.close





    # maxspeed = 0.4
    # if (maxspeed > 0.5):
    #     maxspeed = 0.5
    # if (data.IR_front_right < 50 or data.IR_front_left < 50):
    #     if (twist0.linear.x - 0.01 <= 0):
    #         twist0.linear.x = 0
    #     else:
    #         twist0.linear.x -= 0.01
    # else:
    #     if (twist0.linear.x < maxspeed):
    #         twist0.linear.x += 0.01
    #     else:
    #         twist0.linear.x -= 0.01
    # cmd_vel_publisher.publish(twist0)




class PID:

    def __init__(self, P, I, D, windup_guard):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.ITerm = 0

        self.last_time = time.time()
        self.windup_guard = windup_guard

        self.last_error = 0.0
        self.clear()

    def clear(self):
        self.ITerm = 0.0

    # Returns omega
    def update(self, error, save_state):
        #print("error in update:", error) # DEBUG

        if not save_state:
            old_ITerm      = self.ITerm
            old_last_time  = self.last_time
            old_last_error = self.last_error

        current_time = time.time()
        delta_time = current_time - self.last_time

        delta_error = error - self.last_error

        # Calculate P
        PTerm = self.Kp * error
        
        # Calculate I
        self.ITerm += error * delta_time

        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard

        # Calculate D
        DTerm = 0.0
        if delta_time > 0:
            DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time = current_time
        self.last_error = error

        res = PTerm + (self.Ki * self.ITerm) + (self.Kd * DTerm)

        if not save_state:
            self.ITerm      = old_ITerm
            self.last_time  = old_last_time
            self.last_error = old_last_error

        return res


if __name__ == '__main__': 
    set_delay = False
    PATH = PATHS[sys.argv[1]]
    LOOP_PATH = LOOP_PATHS[sys.argv[1]]
    node = GoToGoalNode()
    
    rospy.on_shutdown(node.shutdown)
    while not rospy.is_shutdown():
        #if (node.i < len(PATH)):
        node.pub_path(PATH[node.i:])
        if node.omega == 0:
            print("=== LOST CONTACT WITH GULLIVIEW! ===")
        if (set_delay):
            delay_duration = random.uniform(1.0, 3.0) #Introduce time delay
            print("Introducing delay of:",delay_duration, "seconds")
            set_delay = False
            time.sleep(delay_duration)

        if (node.sleepTimer <= 0 and node.waitBeforeLooping == False):
            node.move(node.speed, node.omega)
        else:
            #node.move(0, 0)
            print("Sleep timer:", node.sleepTimer, "  Wait before looping:", node.waitBeforeLooping )

            node.sleepTimer =- 1
        node.omega = 0 # safety if loses contact with GulliView
        node.rate.sleep()
        #print("DONE :D")
        
        if node.loopi >= len(LOOP_PATH):
            node.i = 1
            node.loopi = 0
            node.waitBeforeLooping = True


            
