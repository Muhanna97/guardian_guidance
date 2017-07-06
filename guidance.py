#!/usr/bin/env python
# -*- coding: utf-8 -*-
from guidancePKG import vehicle
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Vehicle
print vehicle.mode
# --------------------------------- works with vehicleglobal
# from dronekit import *
# from pymavlink import mavutil # Needed for command message definitions
# import time
# import math
# import datetime as dt
#
# def thing():
#     vehicle = connect("127.0.0.1:14551", wait_ready=True)
#
#     print vehicle.mode
#
#     return vehicle
