# from guidancePKG import vehicle as vehicle
from dronekit import *
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import datetime as dt

# class drone():
#     def __init__(self):
#         self = vehicle = connect("127.0.0.1:14551", wait_ready=True)



def thing():
    vehicle = connect("127.0.0.1:14551", wait_ready=True)
    return vehicle
#
#     print vehicle.mode
