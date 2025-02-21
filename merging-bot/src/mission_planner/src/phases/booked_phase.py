import rospy
from geometry_msgs.msg import Twist

from phases.phase import Phase
import velocity_math as vmath
import math


class BookedPhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)
        self.start_road = mission.start_road
        self.destination_road = mission.destination_road
        self.target_line = mission.stop_line

        self.time = rospy.get_time()
        self.where_in_schema = 0

        self.tw = Twist()
        self.last_twist_send_time = rospy.Time.now()
        self.last_twist = Twist()
        self.step = None
        self.error = None
        self.v_target = None
        self.v_prev = None
        self.t_now = None
        self.t_prev = None

        if self.destination_road.name == 'N':
            self.condition_exp = lambda: self.mission.pos.y <= self.target_line
        elif self.destination_road.name == 'S':
            self.condition_exp = lambda: self.mission.pos.y >= self.target_line
        elif self.destination_road.name == 'E':
            self.condition_exp = lambda: self.mission.pos.x >= self.target_line
        elif self.destination_road.name == 'W':
            self.condition_exp = lambda: self.mission.pos.x <= self.target_line

        if self.destination_road.name == 'N' or self.destination_road.name == 'S':
            self.D = abs(self.mission.pos.y - self.target_line)
        elif self.destination_road.name == 'E' or self.destination_road.name == 'W':
            self.D = abs(self.mission.pos.x - self.target_line)
        self.schema = vmath.calculate_acceleration_schema(self.time, self.mission.new_reservation_start_time,
                                                          0, self.mission.vkors, self.mission.vmax,
                                                          self.mission.acc[1], self.mission.acc[0], self.D)

    @property
    def name(self):
        return "Approach intersection according to booking"

    def begin(self):
        pass

    def run(self):
        self.time = rospy.get_time()
        if self.time > self.schema[self.where_in_schema][0]:
            self.where_in_schema += 1
        self.v_target = self.schema[self.where_in_schema][1]
        self.send_twist()

    def finish(self):
        rospy.loginfo("I have approached the intersection")

    def condition(self):
        return self.condition_exp()

    def ramped_vel(self):
        if self.v_target > self.v_prev:
            self.step = self.mission.acc[1] * (self.t_now - self.t_prev).to_sec()
        else:
            self.step = self.mission.acc[0] * (self.t_now - self.t_prev).to_sec()
        self.error = math.fabs(self.v_target - self.v_prev)
        if self.error < self.step:
            self.v_prev = self.v_target
            return self.v_prev
        else:
            self.v_prev += self.step
            return self.v_prev

    def ramped_twist(self):
        self.tw = Twist()
        self.tw.linear.x = self.ramped_vel()
        return self.tw

    def send_twist(self):
        self.t_now = rospy.Time.now()
        self.last_twist = self.ramped_twist()
        self.t_prev = self.t_now
        self.mission.cmd_vel_pub.publish(self.last_twist)