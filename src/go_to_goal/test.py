#!/bin/env python3

import rospy
from gv_client.msg import GulliViewPosition
from roswifibot.msg import Status
import sys


class Point:
    def __init__(self, id, x, y, theta):
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta
        
    def __str__(self):
        return f'(Id: {self.id}, X: {self.x}, Y: {self.y}, Theta: {self.theta})'


class Bot:
    def __init__(self, id):
        self.id = id
        self.left_speed = 0
        self.right_speed = 0
        self.absolute_speed = 0
        self.point = None
        
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
        self.p = Point(id, x, y, theta)
        if id in bots:
            bots[id].point = self.p
        else: 
            bots[id] = Bot(id)
            bots[id].point = self.p
   
        
    def status(self, status_msg):
        bots[my_id].left_speed = status_msg.speed_front_left
        bots[my_id].left_speed = status_msg.speed_front_right
        bots[my_id].absolute_speed = (bots[my_id].left_speed + bots[my_id].left_speed)/2
        
if __name__ == '__main__': 
    
    my_id = sys.argv[1]
    bots = {my_id:Bot(my_id)}
    
    rospy.init_node('test', anonymous=True)
    rospy.loginfo("Starting test node")
    
    node = Receiver()
    rospy.spin() 
    
    

