#!/usr/bin/env python
# -*- coding: utf-8 -*-
# from dronekit import *
import sys
import time
# sys.path.append('/dronekit-python/dronekit')

from dronekit import *
from dronekit import connect

from pymavlink import mavutil # Needed for command message definitions
import math
import datetime as dt


print dir()

# vehicle = connect("127.0.0.1:14550", wait_ready=True)
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)
