#!/usr/bin/env python3
import threading
import struct
import socket
import os
import sys
import rospy
import pandas as pd
import datetime
import math

from socketserver import BaseRequestHandler, UDPServer
from std_msgs.msg import Header
from gullivutil import parse_packet

time = lambda: datetime.datetime.now()
time_s = lambda: time().timestamp()
time_string = lambda: time().strftime("%Y-%m-%d_%H-%M-%S")
time_string_ms = lambda: time().strftime("%H-%M-%S-%f")

MAX_SPEED = 0.5
MERGING_WINDOW = 3


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f'({self.x}, {self.y})'

class Condition:
    def __init__(self, c):
        self.c = c

    def __str__(self):
        # if true print green, if false print red
        return f'\033[1;32m{self.c}\033[0m' if self.c else f'\033[1;31m{self.c}\033[0m'

    def __repr__(self):
        return self.c

# Class to hold entry detection data
class Detection:
    def __init__(self, name, bot):
        self.name = name
        self.bot = bot
        self.time = time_string_ms()

    def __str__(self):
        return f'Detection {self.name} for bot {self.bot} at {self.time}'

    def update_bot(self, bot):
        # TODO evaluate later
        self.bot.update(bot)
        #coeff = 0.5 # how much of the old point we keep
        # set the new point to be an average between old and new with coeff
        #self.bot.p = Point(self.bot.p.x * coeff + bot.p.x * (1 - coeff), self.bot.p.y * coeff + bot.p.y * (1 - coeff))
        self.time = time_string_ms()

# Detections for main and ramp
class Detections:
    def __init__(self):
        self.main = None
        self.ramp = None

    def __str__(self):
        return f'{self.main if self.main else "No main"}\n{self.ramp if self.ramp else "No ramp"}'

    def has_main(self):
        return self.main is not None

    def has_ramp(self):
        return self.ramp is not None
    
    def has_no_main(self):
        return not self.has_main()

    def has_no_ramp(self):
        return not self.has_ramp()

    def has_both(self):
        return self.has_main() and self.has_ramp()

    def has_main_id(self, bot):
        return self.has_main() and self.main.bot.id == bot.id

    def has_ramp_id(self, bot):
        return self.has_ramp() and self.ramp.bot.id == bot.id

    def get_main_p(self):
        return self.main.bot.p

    def get_ramp_p(self):
        return self.ramp.bot.p

    def get_main_speed(self):
        return self.main.bot.speed

    def get_ramp_speed(self):
        return self.ramp.bot.speed

    def set_main_speed(self, s):
        self.main.bot.set_speed(s)

    def set_ramp_speed(self, s):
        # set_ramp_speed_theta()
        self.ramp.bot.set_speed(s)

    def set_ramp_speed_theta(self, s):
        theta = self.ramp.bot.get_theta()
        # difference in theta and minus half pi
        y_component = s * math.cos(-math.pi*0.5 - theta)
        adjusted_speed = s / y_component
        self.ramp.bot.set_speed(adjusted_speed)
        # change color to red
        print('\033[91m')
        # print debug for all values
        log(f'Adjusted speed: {adjusted_speed}, theta: {theta}, y_component: {y_component}, s: {s}')
        # reset color
        print('\033[0m')

    def reset_speeds(self):
        self.main.bot.reset_speed()
        self.ramp.bot.reset_speed()

    def update(self, detection):
        if detection.name == 'main':
            #self.main = Detection(detection.name, detection.bot.shifted())
            self.main = detection
        elif detection.name == 'ramp':
            self.ramp = detection

    def detect_bot(self, bot):
        if self.has_main_id(bot):
            self.main.update_bot(bot)
        elif self.has_ramp_id(bot):
            self.ramp.update_bot(bot)
        elif self.has_no_main() and bot.is_within_radius_of(main_entry):
            log(f'Bot {bot.id} detected at main entry')
            # set speed for main
            bot.set_speed(0.23)
            self.update(Detection('main', bot))
        elif self.has_no_ramp() and bot.is_within_radius_of(ramp_entry):
            log(f'Bot {bot.id} detected at ramp entry')
            # set speed for ramp
            bot.set_speed(0.17)
            self.update(Detection('ramp', bot))
        else:
            log(bot)

    def pack(self):
        # set printout color to blue
        log('\033[94m')
        log(f'Packing (id: {self.main.bot.id}, speed: {self.main.bot.speed}) and (id: {self.ramp.bot.id}, speed: {self.ramp.bot.speed})')
        # reset printout color
        log('\033[0m')
        return struct.pack('>IfIf', self.main.bot.id, numpy.float32(self.main.bot.speed), self.ramp.bot.id, numpy.float32(self.ramp.bot.speed))

class Bot:
    main_x_shift = 550 # amount to adjust x by to "fake" the bot being in the actual lane for testing
    start_speed = 0.2
    start_theta = 0
    max_safe_speed = 0.5
    min_safe_speed = 0.05

    def __init__(self, id, p, t):
        self.id = id
        self.set_p(p)
        self.set_theta(t)
        self.set_speed(self.start_speed)

    def __str__(self):
        return f'Bot {self.id} at {self.p} with speed {self.speed}'

    def update(self, bot):
        self.set_p(bot.p)
        self.set_theta(bot.theta)

    def set_p(self, p):
        self.p = p

    def set_speed(self, speed):
        if (speed < self.min_safe_speed or speed > self.max_safe_speed):
            # set console text color to red
            print('\033[91m')
            log('*'*32)
            log('_'*32)
            log(f'[!] Safety Cut-off: speed {speed} for bot {self.id}')
            log('_'*32)
            log('*'*32)
            # reset console text color
            print('\033[0m')
        # Safety cut-off
        self.speed = max(min(speed, self.max_safe_speed), self.min_safe_speed)

    def get_theta(self):
        return self.theta

    def set_theta(self, t):
        self.theta = t

    def reset_speed(self):
        self.set_speed(self.start_speed)

    def shifted(self):
        return Bot(self.id, Point(self.p.x + self.main_x_shift, self.p.y), self.theta)

    def is_within_radius_of(self, p):
        return distance_squared(self.p, p) < entry_radius_squared

# Print and log to file
def log(s):
    print(s)
    # create a file named {file_name} inside the folder
    with open(f'logs/{file_name}/{file_name}.txt', 'a') as f:
        f.write(str(s) + '\n')

# Save and print a timestamp for ticks
def timestamp():
    global time_stamp
    old_time = time_stamp if time_stamp != 0 else time_s()
    time_stamp = time_s()
    return min(time_stamp - old_time, 0.3) # safety check to avoid large time and acceleration

# squared distance between two points
def distance_squared(a: Point, b: Point):
    return ((a.x - b.x)**2 + (a.y - b.y)**2)

# distance between two points
def distance(a: Point, b: Point):
    return distance_squared(a, b)**0.5

# distance in meters (coverted from mm), preserve the sign
def distance_in_m(a: Point, b: Point):
    abs_dist = distance_in_m_abs(a, b)
    return abs_dist if a.y > b.y else -abs_dist

def distance_in_m_abs(a: Point, b: Point):
    return distance(a, b) / 1000

def calculation():
    global beta
    if detections.has_both():
        
        x_M = distance_in_m(main_entry, detections.get_main_p()) # x_M - x_0_M
        x_OR = distance_in_m(ramp_entry, detections.get_ramp_p()) # x_OR - x_0_OR

        # abs the d_M and d_OR as an attempt # TODO ask Selpi
        d_M = distance_in_m_abs(detections.get_main_p(), merging_point) # d_0_M - x_M
        d_OR = distance_in_m_abs(detections.get_ramp_p(), merging_point) # d_0_OR - x_OR

        # while-loop conditions
        c1 = Condition(x_M < d_0_M)
        c2 = Condition(x_OR < d_0_OR)
        c3 = Condition(beta < d_f)

        # check all conditions
        if c1.c or c2.c or c3.c:
            # calculate speed

            V_M = detections.main.bot.speed
            V_OR = detections.ramp.bot.speed

            t_f = (d_M + d_OR + d_f) / (V_M + V_OR)

            a_plus = ((d_M - d_OR + d_f) / t_f**2) - ((V_M - V_OR) / t_f)
            a_minus = (-(d_M - d_OR - d_f) / t_f**2) + ((V_M - V_OR) / t_f)

            time_delta = timestamp()

            a_minus_is_less = Condition(a_minus < a_plus)
            if a_minus_is_less.c:
                # ramp accelerates with a_minus and main decelerate with a_minus
                V_OR += a_minus * time_delta * a_coeff
                V_M -= a_minus * time_delta * a_coeff
                beta = x_OR - d_0_OR
            else:
                # ramp decelerates with a_plus and main accelerates with a_plus
                V_OR -= a_plus * time_delta * a_coeff
                V_M += a_plus * time_delta * a_coeff
                beta = x_M - d_0_M

            # set speed
            detections.set_main_speed(V_M)
            detections.set_ramp_speed(V_OR)

            # distance between bots
            dist = distance_in_m_abs(detections.get_main_p(), detections.get_ramp_p())

            # print out all the values formatted nicely
            formatted_time = time_string_ms()
            log(f'\n[Time]            = [{formatted_time}]')
            log(f'[Conditions]      = [({c1}, {c2}, {c3})]')
            log(f'[dist]            = [{dist}]')
            log(f'[d_0_M, d_0_OR]   = [{d_0_M}, {d_0_OR}]')
            log(f'[d_M, d_OR]       = [{d_M}, {d_OR}]')
            log(f'[x_M, x_OR]       = [{x_M}, {x_OR}]')
            log(f'[t_f]             = [{t_f}]')
            log(f'[a_plus, a_minus] = [{a_plus}, {a_minus}]')
            log(f'[a_coeff]         = [{a_coeff}]')
            log(f'[a_minus_is_less] = [{a_minus_is_less}]')
            log(f'[V_M, V_OR]       = [{V_M}, {V_OR}]')
            log(f'[beta]            = [{beta}]')
            log(f'[time_delta]      = [{time_delta}]')

            # log to df
            df.loc[len(df)] = [time(), int(c1.c), int(c2.c), int(c3.c), dist, d_f, d_0_M, d_0_OR, d_M, d_OR, x_M, x_OR, t_f, a_plus, a_minus, a_coeff, a_minus_is_less, V_M, V_OR, beta, time_delta]

        else:
            detections.reset_speeds()
            # set printout color to green
            print('\033[92m')
            log(f'\n[Conditions]    = ({c1}, {c2}, {c3})')
            log(f'[d_0_M, d_0_OR]   = [{d_0_M}, {d_0_OR}]')
            log(f'[d_M, d_OR]       = [{d_M}, {d_OR}]')
            log(f'[beta]            = [{beta}]')
            # reset printout color
            print('\033[0m')

def save_plots():
    if not detections.has_both():
        log('No detections, no plots')
    else:
        path = f'logs/{file_name}/{file_name}'
        # save dataframe to csv in logs folder and enumerate
        df.to_csv(f'{path}.csv')
    
        try:
            plt = df.plot(x='time', y=['c1', 'c2', 'c3'])
            plt.get_figure().savefig(f'{path}_c.png')

            plt = df.plot(x='time', y=['dist', 'd_f'])
            plt.get_figure().savefig(f'{path}_dist.png')

            plt = df.plot(x='time', y=['V_M', 'V_OR'])
            plt.get_figure().savefig(f'{path}_speed.png')
       
            plt = df.plot(x='time', y=['a_plus', 'a_minus'])
            plt.get_figure().savefig(f'{path}_acceleration.png')
       
            plt = df.plot(x='time', y=['beta'])
            plt.get_figure().savefig(f'{path}_beta.png')
       
            plt = df.plot(x='time', y=['d_M', 'd_OR'])
            plt.get_figure().savefig(f'{path}_d.png')
       
            plt = df.plot(x='time', y=['x_M', 'x_OR'])
            plt.get_figure().savefig(f'{path}_x.png')
       
            plt = df.plot(x='time', y=['t_f'])
            plt.get_figure().savefig(f'{path}_t_f.png')

            log('Plots saved')

        except:
            # print error cause in red
            print('\033[91m')
            log(f'Error saving plots: {sys.exc_info()[0]}')
            log(f'Reason: {sys.exc_info()[1]}')
            # reset color
            print('\033[0m')

    
class GulliViewPacketHandler(BaseRequestHandler):
    """
    Request handler to unpack GulliView packets and publish data on a ROS topic.

    The handle() method is called when a 'request' (i.e. UDP packet) is
    received from GulliView on the roof system. The received data is
    unpacked and sent on the ROS topic that ``cls.Publisher`` is publishing on.
    """
    listen_tag_id = None  # Tag ID to listen for

    def handle(self):
        # Receiving binary detection data from GulliView
        recv_buf = bytearray(self.request[0])
        packet = parse_packet(recv_buf)

        for det in packet.detections:
            # If we aren't listening for all tags, and this is not the tag
            # we are listening for, skip it.
            if self.listen_tag_id != "all" and det.tag_id != self.listen_tag_id:
                continue

            header = Header()
            header.stamp = rospy.Time.from_sec(packet.header.timestamp / 1000)

            # print detections
            bot = Bot(det.tag_id, Point(det.x, det.y), det.theta)
            detections.detect_bot(bot)

            log(detections)

            # perform calculations
            calculation()

            # send out over UDP
            self.send_speed()
            log('_'*32)

    # send out dictionary over udp
    def send_speed(self):
        if detections.has_both():
            data = detections.pack()
            sock.sendto(data, ('192.168.50.255', 2222))
        else:
            # set printout color to gray
            print('\033[90m')
            log('No detections, not sending speed')
            # reset printout color
            print('\033[0m')


def new_calculation():
    if not detections.has_both():
        return
    main_speed = MAX_SPEED    
    ramp_speed = MAX_SPEED

    ramp_distance_to_merge = distance_in_m_abs(detections.get_ramp_p(), merging_point)
    main_distance_to_merge = distance_in_m_abs(detections.get_main_p(), merging_point)

    ramp_time_to_merge = ramp_distance_to_merge / detections.ramp.bot.speed 
    main_time_to_merge = main_distance_to_merge / detections.main.bot.speed

    time_gap = abs(ramp_time_to_merge - main_time_to_merge)

    if time_gap < MERGING_WINDOW:

        correction_time = MERGING_WINDOW - time_gap # time vehicles need to increase between them

        if main_time_to_merge <= ramp_time_to_merge: # Main vehicle first
            print("main first")
            main_speed = MAX_SPEED
            ramp_speed = ramp_distance_to_merge / (ramp_time_to_merge + correction_time)
        else:
            ramp_speed = MAX_SPEED
            main_speed = main_distance_to_merge / (main_time_to_merge + correction_time)
            print("ramp first")

    else:
        main_speed = MAX_SPEED
        ramp_speed = MAX_SPEED
    
    detections.set_main_speed(main_speed)
    detections.set_ramp_speed(ramp_speed)


if __name__ == "__main__":
    # vars
    main_entry = Point(912, 8500)
    ramp_entry = Point(2289, 7462)
    merging_point = Point(1021, 4951)
    new_merging_point = Point(1015, 6420)
    entry_radius_squared = 500**2
    
    d_0_M = distance_in_m(main_entry, merging_point)
    d_0_OR = distance_in_m(ramp_entry, merging_point)
    
    beta = 0
    d_f = 1 # 1m = 100cm
    a_coeff = 1
    detections = Detections()
    
    time_stamp = 0

    # Pandas dataframe for logging
    df = pd.DataFrame(columns=['time', 'c1', 'c2', 'c3', 'dist', 'd_f', 'd_0_M', 'd_0_OR', 'd_M', 'd_OR', 'x_M', 'x_OR', 't_f', 'a_plus', 'a_minus', 'a_coeff', 'a_minus_is_less', 'V_M', 'V_OR', 'beta', 'time_delta'])

    rospy.init_node('laptop', anonymous=True)

    file_name = f'log_{time_string()}'
    # create a folder named {file_name} inside logs folder
    if not os.path.exists(f'logs/{file_name}'):
        os.makedirs(f'logs/{file_name}')

    log(f"Started laptop node")

    host = rospy.get_param("~host", default="0.0.0.0")
    port = rospy.get_param("~port", default=2121)
    listen_tag_id = rospy.get_param("~tag_id", default="all")

    # Set static variables on packet handler class to pass information to its instancesmerging_point
    GulliViewPacketHandler.listen_tag_id = listen_tag_id

    log(f"Starting UDP server on {host}:{port}, listening for tag ID: {listen_tag_id}")
    with UDPServer((host, port), GulliViewPacketHandler) as server:
        try:
            server_thread = threading.Thread(target=server.serve_forever)
            server_thread.start()
    
            # Create UDP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
            # Set the socket to broadcast mode
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
            log(f"Started UDP server thread")
            # Spin main thread (the ROS node) until shutdown
            rospy.spin()
    
        except KeyboardInterrupt:
            # Shut down server when node shuts down
            rospy.loginfo("Node received shutdown signal, shutting down server")

        finally:
            # close server
            log("Server shutdown, exiting")
            server.shutdown()

            # close thread
            server_thread.join()

            # close socket
            sock.close()

            # close node
            rospy.signal_shutdown("Node shutdown")

            save_plots()
