#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import datetime as dt
from datetime import timedelta
import mthread
import scheduler
from math import sin, cos, atan2, asin, atan
from math import radians as rad
from math import degrees as deg



#Set up option parsing to get connection string
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
#
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)


    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True


    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)


#Arm and take of to altitude of 5 meters
arm_and_takeoff(30)



"""
Convenience functions for sending immediate/guided mode commands to control the Copter.

The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.


The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

    For more information see:
    http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
    """
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)



"""
Functions to make it easy to convert between the different frames-of-reference. In particular these
make it easy to navigate in terms of "metres from the current position" when using commands that take
absolute positions in decimal degrees.

The methods are approximations only, and may be less accurate over longer distances, and when close
to the Earth's poles.

Specifically, it provides:
* get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
* get_distance_metres - Get the distance between two LocationGlobal objects in metres
* get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing;



"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

The methods include:
* goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
* goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in
    MAV_FRAME_BODY_NED frame
* goto - A convenience function that can use vehicle.simple_goto (default) or
    goto_position_target_global_int to travel to a specific position in metres
    North and East from the current location.
    This method reports distance to the destination.
"""

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see:
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)



def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
    the target position. This allows it to be called with different position-setting commands.
    By default it uses the standard method: dronekit.lib.vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """

    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print "Distance to target: ", remainingDistance
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print "Reached target"
            break;
        time.sleep(2)




"""
Functions that move the vehicle by specifying the velocity components in each direction.
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.

The methods include:
* send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
* send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    # for x in range(0,duration):
    vehicle.send_mavlink(msg)
        # time.sleep(1)




def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def getBearingGood(currentLocation, targetLocation):
    lat1 = rad(currentLocation.lat)
    lat2 = rad(targetLocation.lat)
    long1 = rad(currentLocation.lon)
    long2 = rad(targetLocation.lon)
    deltalong = long2-long1
    return atan2(sin(deltalong)*cos(lat2),cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltalong))

def secondsSince(prevTime):
    timetoCheck = dt.datetime.now() - prevTime
    convertSec = timetoCheck.seconds + (timetoCheck.microseconds/1e6)
    return convertSec


class gotoGuardian3(mthread.MicroThread):
    targetLocation = LocationGlobalRelative(49.130629, -122.793948, 100) # Initializing, will be changed later
    doingMission = 1

    def __init__(self, number):
        self.num = number # for scheduler
        self.runInterval = 0.1 # will run every runInterval seconds (or longer if something blocks) - allows dynamic reschedule
        self.wpNum = 0 # current wp we're flying to
        self.lastRunTime =  dt.datetime.now()

    def createwp(self):
        try:
            gotoGuardian3.targetLocation = LocationGlobalRelative(lats[self.wpNum],longs[self.wpNum],alts[self.wpNum])
        except IndexError:
            if gotoGuardian3.doingMission == 1:
                # only set to RTL once in case we want to manually take over after the iniaial RTL command
                vehicle.mode = VehicleMode("RTL") # RTL if we're out of waypoints. Kind of nasty but it'll work
                print "No more waypoints, RTL"
            self.runInterval = 5
            gotoGuardian3.doingMission = 0

    def doMission(self):
        # before sending commands to go somewhere, check if we can do mission
        # for example, if we have been told to RTL by the pilot, don't try to go anywhere
        if gotoGuardian3.doingMission == 0:
            print vehicle.mode
            return 0
        elif (vehicle.mode == "GUIDED"):
            gotoGuardian3.doingMission = 1
            return 1
        else:
            gotoGuardian3.doingMission = 0
            print vehicle.mode
            return 0

    def step(self):
        stepTimer = secondsSince(self.lastRunTime)
        if (stepTimer >= self.runInterval):
            print  "\r\n"

            self.createwp() # create the current target wp
            if self.doMission(): # if we can do mission (not in RTL or some other case)
                self.go() # go to the target location

            if(self.checkIfPointReached(gotoGuardian3.targetLocation)):
                self.wpNum += 1

            self.lastRunTime = dt.datetime.now()

    def checkIfPointReached(self,targetLocation):
        wpDistance = get_distance_metres(vehicle.location.global_relative_frame, gotoGuardian3.targetLocation)
        # if(wpDistance <= targetDistance*0.01 or wpDistance < 0.3):
        #     return 1
        # else:
        #     return 0
        if(wpDistance < 1):
            return 1
        else:
            return 0

    def go(self):
        targetLocation = gotoGuardian3.targetLocation
        currentLocation = vehicle.location.global_relative_frame
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        print "Going to waypoint: ", (self.wpNum+1) # human format (1 indexed)

        if (targetDistance > 20):
            if not checkObstacle.obstacleFound: # we good
                print "Velocity control"
                bearing = getBearingGood(vehicle.location.global_relative_frame,targetLocation)
            else:
                print "Velocity control - obstacle avoidance"
                obstacleBearing = deg(getBearingGood(vehicle.location.global_relative_frame,checkObstacle.obstacleLocation))
                if checkObstacle.obstacleLocation.lat >= targetLocation.lat:
                    bearing = rad(obstacleBearing - 90)
                else:
                    bearing = rad(obstacleBearing + 90)

            # Velocity vector creation
            northScale = cos(bearing)
            eastScale = sin(bearing)

            print "Flight bearing ", deg(bearing)
            condition_yaw(deg(bearing)) #point towards travel direction

            northVelocity = defaultVelocity*northScale
            eastVelocity = defaultVelocity*eastScale

            print "Velocity components:", northVelocity," ,", eastVelocity
            print "Distance to target:", targetDistance
            print "Obstacle bearing: ", checkObstacle.obstacleBearing
            print "Obstacle is ", checkObstacle.obstacleDistance, "metres away"

            send_ned_velocity(northVelocity,eastVelocity,0)
            return 0

        elif(targetDistance <= 20): # if close to target, go right there
            print "Going straight to target"
            print "Distance to target: ", targetDistance
            vehicle.simple_goto(targetLocation)
            return 0

class checkObstacle(mthread.MicroThread):
    obstacleFound = 0
    obstacleLocation = LocationGlobalRelative(49.128400, -122.798044, 30)
    obstacleBearing = 0
    obstacleDistance = 0
    def __init__(self, number):
        self.num = number
        self.lastRunTime =  dt.datetime.now()
        self.interceptBearing = 0
        self.runInterval = 0.1 # will run every runInterval seconds (or longer if something blocks) - allows dynamic reschedule
        self.avoidRadius = 50

    def step(self):
        timeSince = secondsSince(self.lastRunTime)
        if (timeSince >= self.runInterval):

            currentLocation = vehicle.location.global_relative_frame
            checkObstacle.obstacleDistance = get_distance_metres(currentLocation, checkObstacle.obstacleLocation)
            checkObstacle.obstacleBearing = deg(getBearingGood(currentLocation, checkObstacle.obstacleLocation))
            self.obstacleTargetDistance = get_distance_metres(checkObstacle.obstacleLocation, gotoGuardian3.targetLocation)

            if self.canAvoid(): # only avoid obstacles if appropriate
                if (checkObstacle.obstacleDistance <= self.avoidRadius):
                    if self.interceptBearing == 0:
                        self.interceptBearing = checkObstacle.obstacleBearing
                    checkObstacle.obstacleFound = 1

                elif(checkObstacle.obstacleFound == 1 and (abs(checkObstacle.obstacleBearing - self.interceptBearing) >= 180)): # breakout condition - we have gone over 180 degrees around the obstacle
                    checkObstacle.obstacleFound = 0
                else:
                    checkObstacle.obstacleFound = 0

                self.lastRunTime = dt.datetime.now()

    def canAvoid(self):
        # we want to skip obstacle avoidance in RTL mode (because it might be a failsafe)
        # also skip if waypoint is within obstacle avoid radius. Would rather hit obstacle than miss waypoint.
        if gotoGuardian3.doingMission == 0:
            return 0
        if vehicle.mode == "RTL":
            return 0
        elif  self.obstacleTargetDistance <= self.avoidRadius:
            print "Obstacle is close to waypoint. Proceeding to waypoint. Collision possible."
            print "Obstacle to waypoint distance:", self.obstacleTargetDistance
            return 0
        else:
            return 1



# ---------------------------- GUARDIAN -------------------------------

# lats = [49.1294287000000,49.1295059410421 ,49.1293421175351,49.1292871281073,49.1291455562146,49.1291782940281,49.1290144705210,49.1290039843219,49.1288624124292,49.1288506470140,49.1286868235070,49.1287208405365,49.1285792686438,49.1285230000000];
# longs = [-122.796303000000,-122.790660529535,-122.790680057946, -122.796304799855,-122.796306599709,-122.790699586357,-122.790719114768,-122.796308399564,-122.796310199418,-122.790738643178,-122.790758171589,-122.796311999273,-122.796313799127,-122.790777700000];
# alts = [30,30,30,30,30,30,30,30,30,30,30,30,30,30];

# lats = [49.1294287000000,49.1295059410421,49.1293421175351];
# longs = [-122.796303000000,-122.790660529535,-122.790680057946];
lats = [49.128695,49.1294287000000]
longs = [-122.800132, -122.796303000000]
alts = [30,30];

defaultVelocity = 5
# guidanceFrequency = 10 #Hz - number of times the guidance loop runs per second
# obstacleCheckFrequency = 10 #Hz - number of times proximity to an obstacle is checked

print "Team Guardian AUVSI Guidance Script"

# create cooperatively scheduled threads
mt2 = gotoGuardian3(1)
mt3 = checkObstacle(2)

ms = scheduler.Scheduler()
ms.add_microthread(mt2)
ms.add_microthread(mt3)

# run the threads - currently no exit from this, so LAND and CLOSE section will not be reached
# ctrl+c the script to kill it
ms.run()



# -------------------- LAND and CLOSE -----------------------

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed mission. Thank you!")
