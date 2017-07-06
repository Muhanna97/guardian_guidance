#!/usr/bin/env python
# -*- coding: utf-8 -*-
from guidancePKG import vehicle
from guidancePKG import arm_and_takeoff
print vehicle.altitude

arm_and_takeoff(30)

print vehicle.altitude
