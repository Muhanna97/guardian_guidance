from dronekit import *
import userDefines
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import datetime as dt

# connects to the port listed. Assumes that SITL is already running, or MAVProxy is already running with a real drone connected
# Author: Richard Arthurs


def makeConnection():
    vehicle = connect(userDefines.UDPPort, wait_ready=True)
    return vehicle
