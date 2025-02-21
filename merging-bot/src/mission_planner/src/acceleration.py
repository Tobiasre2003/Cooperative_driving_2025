import rospy
import math
import time
import sys
from socket import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class acceleration:
    
    def __init__(self):
        self.vel = 0.75
        self.last_twist_send_time = rospy.Time.now()
        self.last_twist = None

        self.target_twist = Twist()
        self.last_twist = Twist()

        self.rate = rospy.Rate(20)

    # Use of this class by setting target_twist.lenear. x or y to a float
    # Then inside a while loop use send_twist()

    def ramped_vel(self):
     self.step = self.ramp_rate * (self.t_now - self.t_prev).to_sec()
     self.sign = 1.0 if (self.v_target > self.v_prev) else -1.0
     self.error = math.fabs(self.v_target - self.v_prev)
     if self.error < self.step:
        return self.v_target
     else: 
        return self.v_prev + self.sign * self.step

    def ramped_twist(self):
     self.tw = Twist()
     self.tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, vel)
     return self.tw

    def send_twist(self):
     self.t_now = rospy.Time.now()
     self.last_twist = ramped_twist(last_twist, target_twist, last_twist_send_time,t_now, vel)
     self.last_twist_send_time = t_now
     self.publisher.publish(last_twist)
