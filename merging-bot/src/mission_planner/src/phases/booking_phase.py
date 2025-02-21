import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from phases.phase import Phase
import velocity_math as vmath


class BookingPhase(Phase):

    def __init__(self, mission: "MissionPlannerNode"):
        super().__init__(mission)

        self.start_road = mission.start_road
        self.destination_road = mission.destination_road
        self.target_line = mission.start_line
        self.end_line = mission.stop_line

        self.acc_min = mission.acc[0]
        self.acc_max = mission.acc[1]
        self.v_kors = mission.vkors
        self.v_max = mission.vmax

        self.t0 = None
        self.v0 = None
        self.D = None

        self.booking = None

        self.enter_velocity = None
        self.schemaEntering = None
        self.exit_velocity  = None
        self.schemaCrossing = None

        self.booking_start = None
        self.booking_end  = None
        self.wished_booking = None
        self.wished_crossing_schema = None
        self.wished_entering_schema = None
        
        self.intersection_length = abs(self.end_line - self.target_line)
    @property
    def name(self):
        return "Booking a spot in intersection"
        
    def begin(self):
        # Get the distance from robot position to stop line to enter the intersection
        if self.destination_road.name == 'N' or self.destination_road.name == 'S':
            self.D = abs(self.mission.pos.y - self.target_line)
        elif self.destination_road.name == 'E' or self.destination_road.name == 'W':
            self.D = abs(self.mission.pos.x - self.target_line)
            
        self.t0 = rospy.get_time()
        self.v0 = self.mission.drive_speed

        self.booking_start, self.enter_velocity, self.schemaEntering  = vmath.calculate_optimal_booking(self.t0, self.v0, self.v_kors, self.v_max, self.acc_min, self.acc_max, self.D)

        self.booking_end, self.exit_velocity, self.schemaCrossing = vmath.calculate_optimal_booking(self.booking_start, self.enter_velocity, self.v_kors, self.v_kors, self.acc_min, self.acc_max, abs(self.end_line - self.target_line))
        
        # Adding safety-margin
        self.booking_start -= (self.booking_start - self.t0) * 0.25
        self.booking_end += (self.booking_end - self.booking_start) * 0.25

        # Send the optimal reservation [self.booking_start,self.booking_end] somewhere
        self.wished_booking = [self.booking_start, self.booking_end]
        self.wished_entering_schema = self.schemaEntering
        self.wished_crossing_schema = self.schemaCrossing
        
        self.booking = Float32MultiArray()

        self.booking.data = [self.booking_start, self.booking_end]
        self.mission.res_pub.publish(self.booking)

    def run(self):
        #Do nothing
        pass

    def finish(self):
        rospy.loginfo("Booking phase is done!")
        pass

    def condition(self):
        return self.mission.go