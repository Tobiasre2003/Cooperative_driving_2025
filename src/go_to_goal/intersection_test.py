#!/usr/bin/env python3
import threading
import struct
import os
import socket
import sys
import pandas as pd
import rospy
import datetime
import math
import argparse
from time import sleep
from collections import deque
from socketserver import BaseRequestHandler, UDPServer
from gv_client.msg import GulliViewPosition
from roswifibot.msg import Status


time = lambda: datetime.datetime.now()
time_s = lambda: time().timestamp()
time_string = lambda: time().strftime("%Y-%m-%d_%H-%M-%S")
time_string_ms = lambda: time().strftime("%H-%M-%S-%f")

# Global variables used for speed and the time between merging vehicles
MAX_SPEED = 0.3
START_SPEED = 0.3
length_of_wifibot = 0.44
MERGING_WINDOW = length_of_wifibot/MAX_SPEED + 3
MAIN_START_SPEED = 0.3
RAMP_START_SPEED = 0.3

INTERSECTION_WINDOW = 6 + length_of_wifibot/MAX_SPEED

#--------------------------------------------------------------

# class Point:
#     def __init__(self, id, x, y, theta):
#         self.id = id
#         self.x = x
#         self.y = y
#         self.theta = theta
        
#     def __str__(self):
#         return f'(Id: {self.id}, X: {self.x}, Y: {self.y}, Theta: {self.theta})'

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f'({self.x}, {self.y})'
    
#------------------------------------------------------------------    
    
    
    
class Bot:
    def __init__(self, id, point, theta):
        self.id = id
        # self.left_speed = 0
        # self.right_speed = 0
        # self.absolute_speed = 0
        self.speed = self.set_speed(self.start_speed)
        self.set_point 
        self.set_theta(theta)
        
    def __str__(self):
        return f'(Id: {self.id}, Left speed: {self.left_speed}, Right speed: {self.right_speed}, Absolute speed: {self.absolute_speed}, Point: {self.point})'
    
    #main_x_shift = 0 # amount to adjust x by to "fake" the bot being in the actual lane for testing
    start_speed = START_SPEED
    start_theta = 0
    max_safe_speed = MAX_SPEED
    min_safe_speed = 0

    restart = False

    def __init__(self, id, p, t):
        self.id = id
        self.set_p(p)
        self.set_theta(t)
        self.set_speed(self.start_speed)

        self.set_restart(False)

    def update(self, bot):
        self.set_point(bot.point)
        self.set_theta(bot.theta)

    def set_point(self, point):
        self.point = point

    def set_speed(self, speed):
        if (speed < self.min_safe_speed or speed > self.max_safe_speed):
            # set console text color to red
            print('\033[91m')
            log('*'*32)
            log('_'*32)
            log(f'[!]  Cut-off: speed {speed} for bot {self.id}')
            log('_'*32)
            log('*'*32)
            # reset console text color
            print('\033[0m')
        # Safety cut-off
        self.speed = max(min(speed, self.max_safe_speed), self.min_safe_speed)

    def set_restart(self, restart):
        self.restart = restart

    def get_theta(self):
        return self.theta

    def set_theta(self, theta):
        self.theta = theta

    def reset_speed(self):
        self.set_speed(self.start_speed)

    # def shifted(self):
    #     return Bot(self.id, Point(self.point.x + self.main_x_shift, self.point.y), self.theta)

    def is_within_radius_of(self, point):
        return distance_squared(self.point, point) < entry_radius_squared

#--------------------------------------------------------------



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

#------------------------------------------------------------------       






# Detections for main and ramp
class Detections:
    def __init__(self):
        self.main = None
        self.ramp = None
        self.intersectionQueue = deque()

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
        log(f'SET SPEED MAIN: {s}')
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

    # used to not send new speeds after the merging point
    def reset(self):
        self.main = None
        self.ramp = None

    # def set_ramp_restart(self, e):
    #     self.ramp.bot.set_restart(e)

    # def set_main_restart(self, e):
    #     self.main.bot.set_restart(e)

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
            bot.set_speed(MAIN_START_SPEED)
            self.update(Detection('main', bot))
        elif self.has_no_ramp() and bot.is_within_radius_of(ramp_entry):
            log(f'Bot {bot.id} detected at ramp entry')
            # set speed for ramp
            bot.set_speed(RAMP_START_SPEED)
            self.update(Detection('ramp', bot))
        else:
            log(bot)
    
    def is_bot_in_intersection(self, bot):
        return distance_squared(bot.p, INTERSECTION_CENTER) < 1000**2

    def pack(self):
        # set printout color to blue
        log('\033[94m')
        log(f'Packing (id: {self.main.bot.id}, speed: {self.main.bot.speed}), RESTART {self.main.bot.restart}),  and (id: {self.ramp.bot.id}, speed: {self.ramp.bot.speed}), RESTART {self.ramp.bot.restart}),  ')
        
        # reset printout color
        log('\033[0m')
        return struct.pack('>If?If?', self.main.bot.id, self.main.bot.speed, self.main.bot.restart, self.ramp.bot.id, self.ramp.bot.speed, self.ramp.bot.restart)

#-------------------------------------------------------------------


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
            
def normalize_time():
    for i in range(1, len(df.time)):
        df.time[i] = df.time[i] - df.time[0]
    df.time[0] = 0

# def intersection_plot():
#     normalize_time()

#     path = f'logs/{file_name}/{file_name}'
#     # save dataframe to csv in logs folder and enumerate
#     df.to_csv(f'{path}.csv')
#     try:
#         plt = df.plot(x = 'time', y = 'ttc', xlabel='Time', ylabel='TTC', title='TTC')
#         plt.get_figure().savefig(f'{path}_TTC.png')

#         plt = df.plot(x= 'time', y=['main_speed', 'ramp_speed'], ylabel = 'Hastighet [m/s]', xlabel='Tid [s]')
#         plt.get_figure().savefig(f'{path}_speeds.png')
#     except Exception as e:
#         log(e)

# def save_plots():
#     if not detections.has_both():
#         log('No detections, no plots')
#     else:
#         normalize_time()

#         path = f'logs/{file_name}/{file_name}'
#         # save dataframe to csv in logs folder and enumerate
#         df.to_csv(f'{path}.csv')
    
#         try:
#             plt = df.plot(x='time', y=['main_speed', 'ramp_speed'], grid=True, xlabel="Time [s]", ylabel='Speed [m/s]', title = "Speeds")
#             plt.get_figure().savefig(f'{path}_speeds.png')

#             plt = df.plot(x='time', y=['main_distance_to_merge', 'ramp_distance_to_merge'], grid=True, xlabel = 'Time [s]', ylabel='Meters', title = "Distance to merge")
#             plt.get_figure().savefig(f'{path}_distances.png')

#             plt = df.plot(x='time', y = ['main_time_to_merge', 'ramp_time_to_merge', 'time_gap'], grid=True, xlabel = "Time [s]", ylabel='Seconds', title = "Time to merge")
#             plt.get_figure().savefig(f'{path}_time_to_merge.png')

#             plt = df.plot(x = 'time', y = ['CRI'], grid=True, xlabel = "Time [s]", ylabel='CRI', title = "CRI")
#             plt.get_figure().savefig(f'{path}_cri.png')

#             log('Plots saved')

#         except:
#             # print error cause in red
#             print('\033[91m')
#             log(f'Error saving plots: {sys.exc_info()[0]}')
#             log(f'Reason: {sys.exc_info()[1]}')
#             # reset color
#             print('\033[0m')


#-----------------------------------------------------------------------

class Receiver:
    def __init__(self):
        rospy.Subscriber('gv_positions', GulliViewPosition, self.positions)
        rospy.Subscriber('status', Status, self.status)
    def get_data(position_msg, status_msg):
        bot = Bot(position_msg.tag_id, Point(position_msg.x, position_msg.y), position_msg.theta)
        detections.detect_bot(bot)
        if detections.has_both():# and len(packet.detections) == 2:
            log(detections)
            merge_done = False

            # perform calculations
            if args.algorithm == 'intersection':
                calculation_intersection()
            else:
                merge_done = calculation()

            # send out over UDP
            self.send_speed()
            log('_'*32)

            # reset detections if both have left the merge
            if(merge_done):
                #save_plots()
                detections.reset()
        else:
            log("Not enough detections")

# Calculates the speed the vehicles need to achieve to keep a safe distance
def calculation():
    # bool that gets returned True when the merge is done
    merge_done = False

    # No detections, do nothing
    if not detections.has_both():
        return merge_done

    # get positions from the robots  
    ramp_pos = detections.get_ramp_p()
    main_pos = detections.get_main_p()

    # Get the current speed that has been sent to the robots
    # TODO get actual data from robots 
    main_speed = detections.main.bot.speed
    ramp_speed = detections.ramp.bot.speed

    main_passed_merge = main_pos.y < MERGING_END.y
    ramp_passed_merge = ramp_pos.y < MERGING_END.y

    # Get current distances to the merging Point
    # since ramp is not travelling straight toward the merge point a point at the end of the ramp is used until
    # it has passed that point
    if ramp_pos.y < MERGING_START.y:
        ramp_distance_to_merge = distance_in_m(ramp_pos, MERGING_END)
    else:
        ramp_distance_to_merge = distance_in_m(ramp_pos, MERGING_START) + ramp_length
    main_distance_to_merge = distance_in_m(main_pos, MERGING_END)

    # Get the time they have left until they are at the merging point
    ramp_time_to_merge = ramp_distance_to_merge / ramp_speed
    main_time_to_merge = main_distance_to_merge / main_speed

    # Time gap between the robots
    time_gap = abs(ramp_time_to_merge - main_time_to_merge)
    print("Timegap between robots: ", time_gap)

    #cri calculations
    d_b = abs(ramp_distance_to_merge - main_distance_to_merge) - length_of_wifibot
    d_a = 1.5
    k_b = d_b/(d_a + d_b)
    if k_b <= 0:
        cri = 1
    else: cri = math.e**(-k_b)
    
    # Set speeds when the bots pass merging point
    if main_passed_merge and ramp_passed_merge:
        detections.reset_speeds()
        log("merge done")
        merge_done = True
        df.loc[len(df)] = [time_s(), main_pos, ramp_pos, main_speed, ramp_speed, main_distance_to_merge, ramp_distance_to_merge, main_time_to_merge, ramp_time_to_merge, 0, time_gap, cri]
        return merge_done
    #only main has passed
    elif main_passed_merge:
        #main_speed = min(MAX_SPEED, main_speed * 1.005)
        main_speed = MAX_SPEED
        detections.set_main_speed(main_speed)
        df.loc[len(df)] = [time_s(), main_pos, ramp_pos, main_speed, ramp_speed, main_distance_to_merge, ramp_distance_to_merge, main_time_to_merge, ramp_time_to_merge, 0, time_gap, cri]
        return merge_done
    #only ramp has passed
    elif ramp_passed_merge:
        #ramp_speed = min(MAX_SPEED, ramp_speed * 1.005)
        ramp_speed = MAX_SPEED
        detections.set_ramp_speed(ramp_speed)
        df.loc[len(df)] = [time_s(), main_pos, ramp_pos, main_speed, ramp_speed, main_distance_to_merge, ramp_distance_to_merge, main_time_to_merge, ramp_time_to_merge, 0, time_gap, cri]
        return merge_done

    
    if time_gap < MERGING_WINDOW:
        # time vehicles need to increase between them
        correction_time = MERGING_WINDOW - time_gap 
        
        # Main robot is first
        if main_time_to_merge <= ramp_time_to_merge:
            #ramp_speed = max(ramp_distance_to_merge / (ramp_time_to_merge + correction_time), ramp_speed * 0.995)
            ramp_speed = ramp_distance_to_merge / (ramp_time_to_merge + correction_time)
            #main_speed = min(MAX_SPEED, main_speed * 1.005)
            main_speed = MAX_SPEED

        # Ramp robot is first
        else:
            #ramp_speed = min(MAX_SPEED, ramp_speed * 1.005)
            #main_speed = max(main_distance_to_merge / (main_time_to_merge + correction_time), main_speed * 0.995) 
            ramp_speed = MAX_SPEED 
            main_speed = main_distance_to_merge / (main_time_to_merge + correction_time)
  

    
    # Speed up if time gap is large enough
    else:
        correction_time = time_gap - MERGING_WINDOW
        if main_time_to_merge <= ramp_time_to_merge:
            #main_speed = min(MAX_SPEED, main_speed * 1.005)
            #ramp_speed = min(ramp_distance_to_merge / (ramp_time_to_merge - correction_time), ramp_speed * 1.005)
            main_speed = MAX_SPEED
            ramp_speed = ramp_distance_to_merge / (ramp_time_to_merge - correction_time)
        else:
            #ramp_speed = min(MAX_SPEED, ramp_speed * 1.005)
            #main_speed = min(main_distance_to_merge / (main_time_to_merge - correction_time), main_speed * 1.005)
            ramp_speed = MAX_SPEED
            main_speed = main_distance_to_merge / (main_time_to_merge - correction_time)

    # set the speeds that will be transmitted    
    detections.set_main_speed(main_speed)
    detections.set_ramp_speed(ramp_speed)

    df.loc[len(df)] = [time_s(), main_pos, ramp_pos, detections.get_main_speed(), detections.get_ramp_speed(), 
                       main_distance_to_merge, ramp_distance_to_merge, main_time_to_merge, ramp_time_to_merge, 
                       correction_time, time_gap, cri]
    return merge_done





if __name__ == "__main__":
    #allows passing of arguments to choose which algorithm to use
    parser = argparse.ArgumentParser(description='Choose an algorithm to run.')
    parser.add_argument('algorithm', choices=('merge', 'intersection'), nargs='?',
                        default='merge', help='Which algorithm to run, default = merge')
    args = parser.parse_args()

    # vars
    main_entry = Point(895,8209)
    ramp_entry = Point(2595,7532)
    
    # Points for start and end of merging zone
    MERGING_START = Point(1530, 6450)
    MERGING_POINT = Point(1100, 4700)
    MERGING_END = MERGING_POINT
    ramp_length = distance_in_m_abs(MERGING_START, MERGING_END)
    # Point for intersection
    if sys.argv[1] == "intersection": # For running intersection in a figure of eight from "merging-bot/merging2024 project"
        INTERSECTION_CENTER = Point(2200, 6800) # ändra  
        MAIN_INTERSECTION_START = Point(2200, 9000) # ändra
        RAMP_INTERSECTION_START = Point(2200, 4600) # ändra
    if sys.argv[1] == "intersection_testet": # For running intersection in "merging2025bachelor project"
        INTERSECTION_CENTER = Point(3628, 3714) # ny
        MAIN_INTERSECTION_START = Point(3918, 7741) # ny
        RAMP_INTERSECTION_START = Point(3487, 805) # ny


    

    # Defina entry radius
    entry_radius_squared = 500**2 

    #entry_radius_squared = 900**2 



    # if argument intersection is passed when running code change where to detect bots.
    if args.algorithm == 'intersection' or args.algorithm == 'intersection_testet':
        main_entry = MAIN_INTERSECTION_START
        ramp_entry = RAMP_INTERSECTION_START
       

        # Pandas dataframe for logging intersection
        df = pd.DataFrame(columns=['time', 'ttc', 'main_speed', 'ramp_speed'])
    else:
        # Pandas dataframe for logging merge
        df = pd.DataFrame(columns=['time', 'main_pos', 'ramp_pos', 'main_speed', 'ramp_speed','main_distance_to_merge', 
                               'ramp_distance_to_merge', 'main_time_to_merge', 'ramp_time_to_merge', 'correction_time', 'time_gap', 'CRI'])

    detections = Detections()

    time_stamp = 0



    file_name = f'log_{time_string()}'
    # create a folder named {file_name} inside logs folderis_main_within_intersection_radius
    if not os.path.exists(f'logs/{file_name}'):
        os.makedirs(f'logs/{file_name}')

    log(f"Started laptop node")

    host = "0.0.0.0"
    port = 2121
    listen_tag_id = "all"
