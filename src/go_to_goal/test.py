#!/bin/env python3

import rospy
from gv_client.msg import GulliViewPosition
from roswifibot.msg import Status


class Point:
    def __init__(self, id, x, y, theta):
        self.id = id
        self.x = x
        self.y = y
        self.theta = theta
        
    def __str__(self):
        return f'(Id: {self.id}, X: {self.x}, Y: {self.y}, Theta: {self.theta})'


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
        print(self.p)
        
    def status(self, status_msg):
        print(status_msg.speed_front_left)
        print(status_msg.speed_front_right)

if __name__ == '__main__': 
    rospy.init_node('test', anonymous=True)
    rospy.loginfo("Starting test node")
    
    node = Receiver()
    rospy.spin() 
    
    

