#!/bin/env python3

import rospy
from gv_client.msg import GulliViewPosition

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
        rospy.init_node('test', anonymous=True)
        rospy.loginfo("Starting test node")
        rospy.Subscriber('gv_positions', GulliViewPosition, self.positions)
    
    def positions(self, position_msg):
        id = position_msg.tagId
        x = position_msg.x
        y = position_msg.y
        theta = position_msg.theta
        self.p = Point(id, x, y, theta)
        print(self.p)


if __name__ == '__main__': 
    node = Receiver()
    rospy.spin()
    
    