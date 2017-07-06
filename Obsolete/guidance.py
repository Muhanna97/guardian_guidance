#!/usr/bin/env python
# -*- coding: utf-8 -*-
from guidancePKG import vehicle
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Vehicle

import vehicleFunction
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


print vehicle
print vehicle.mode

vehicleFunction.vehicleFunction(mydrone, 30)
