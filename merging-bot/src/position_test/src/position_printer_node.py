#!/bin/env python3

# Adapted from mission_planner_node.py

from collections import namedtuple
from typing      import Optional, Callable

import rospy

from std_msgs.msg  import Empty
from gv_client.msg import GulliViewPosition

Position = namedtuple('Position', ['x', 'y', 'theta'])

class PositionPrinterNode:
    def __init__(self):
        rospy.init_node('position_printer_node', anonymous=True)
        rospy.loginfo("Starting position & orientation printer node")

        rospy.Subscriber('gv_positions', GulliViewPosition, self._position_cb)

        self.exit_pub = rospy.Publisher('exit', Empty, queue_size=1)
        self.stopped_pub = rospy.Publisher('stopped', Empty, queue_size=1)

        self.pos: Optional[Position] = None

        # Wait for initial position data
        try:
            rospy.loginfo("Awaiting initial position & orientation data")
            self._await(self._initial_position_received, rate=20, timeout=30.0)
        except TimeoutError as e:
            rospy.logerr("Timeout while waiting for initial position & orientation")
            raise e
        rospy.loginfo(f"Initial position & orientation received: ({self.pos.x}, {self.pos.y}, {self.pos.theta})")

    @staticmethod
    def _await(condition: Callable[[], bool], rate: int = 2, timeout: float = 10.0):
        """Block until a given condition is True.

        Raises TimeoutError if condition does not become true before timeout.

        :param condition: A callable taking no arguments and returning a falsy value
                          while the condition is not met, and a truthy value once it is.
        :param rate: The frequency in Hz at which the condition should be polled
        :param timeout: Number of seconds before action times out.
        """
        rate = rospy.Rate(rate)
        timeout = rospy.Duration(secs=timeout)

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < timeout and not rospy.is_shutdown():
            if condition():
                return
            rate.sleep()
        raise TimeoutError()

    def _initial_position_received(self) -> bool:
        return self.pos is not None

    def _position_cb(self, position_msg):
        self.pos = Position(position_msg.x, position_msg.y, position_msg.theta)

if __name__ == '__main__':
    position_printer = PositionPrinterNode()
