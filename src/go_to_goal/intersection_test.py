#!/usr/bin/env python3
import threading
import struct
import socket
import os
import sys
import pandas as pd
import datetime
import math
import argparse
from time import sleep
from collections import deque
from socketserver import BaseRequestHandler, UDPServer
from gullivutil import parse_packet

time = lambda: datetime.datetime.now()
time_s = lambda: time().timestamp()
time_string = lambda: time().strftime("%Y-%m-%d_%H-%M-%S")
time_string_ms = lambda: time().strftime("%H-%M-%S-%f")

# Global variables used for speed and the time between merging vehicles
MAX_SPEED = 0.3
length_of_wifibot = 0.44
MERGING_WINDOW = length_of_wifibot/MAX_SPEED + 3
MAIN_START_SPEED = 0.3
RAMP_START_SPEED = 0.3

INTERSECTION_WINDOW = 6 + length_of_wifibot/MAX_SPEED