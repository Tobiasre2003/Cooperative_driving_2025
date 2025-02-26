#!/bin/env python3

import rospy
from gv_client.msg import GulliViewPosition

class Test:
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        rospy.loginfo("Starting test node")
        rospy.Subscriber('gv_positions', GulliViewPosition, self._position_cb)

    def _position_cb(self, position_msg):
        self.x = position_msg.x
        self.y = position_msg.y
        self.theta = position_msg.theta
        self.id = position_msg.tagId
        print(self.x,self.y,self.theta, self.id)


if __name__ == '__main__': 
    node = Test()
    rospy.spin()
    
    