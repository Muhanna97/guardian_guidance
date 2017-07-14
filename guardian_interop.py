from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Vehicle, Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import datetime as dt
from datetime import timedelta
import mthread
import json

from math import sin, cos, atan2, asin, atan
from math import radians as rad
from math import degrees as deg
import vehicleFunctions as dk
from userDefines import * # waypoints, default velocity

# Functions and classes written specifically for guardian obstacle avoidance
# Author: Richard Arthurs

def drop_guided(vehicle):
# sends a command to drop the bottle in guided mode.
# This is preferred since it doesn't require a mode change.
    print "Dropping bottle!"
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        servoNum,    # param 1, yaw in degrees
        servoOpen,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
    vehicle.send_mavlink(msg)

def drop_auto(vehicle):
    cmds = vehicle.commands # get current commands
    cmds.clear()
    cmds.upload()
            # the 4th param might need to be 0
            # it doesn't like that the servo open is over 255
    cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, servoNum, servoOpen, 0, 0, 0, 0, 0, 0, 0)

    cmds.add(cmd1)
    cmds.upload() # Send commands
    print "Changing mode to AUTO"
    vehicle.mode = VehicleMode("AUTO")
    print "Dropping bottle!"
    print "Changing mode to GUIDED"
    vehicle.mode = VehicleMode("GUIDED")


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

    def __init__(self, number, vehicleIn):
        self.num = number # for scheduler
        self.runInterval = float(1)/guidanceFrequency # will run every runInterval seconds (or longer if something blocks) - allows dynamic reschedule
        self.wpNum = 0 # current wp we're flying to
        self.lastRunTime =  dt.datetime.now()
        self.vehicle = vehicleIn
        self.dropped = 0
        self.pause = 0

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

    def createwp(self):
        with open('surrey_mission.json') as data_file_one: #chage file name to missions for use
            waypoint_data = json.load(data_file_one)
        inc1 = 0
        mission_dict = {}
        mission_lat = []
        mission_lon = []
        mission_alt = []
        while (inc1 < 3):
            inc2 = 0
            while (inc2 < len(waypoint_data["mission_waypoints"])):
                if (inc1 == 0):
                    lat = waypoint_data["mission_waypoints"][inc2]["latitude"]
                    mission_lat.append(lat)
                if (inc1 == 1):
                    lon = waypoint_data["mission_waypoints"][inc2]["longitude"]
                    mission_lon.append(lon)
                if (inc1 == 2):
                    alt = waypoint_data["mission_waypoints"][inc2]["altitude_msl"]
                    mission_alt.append(alt)
                inc2 = inc2 + 1
            inc1 = inc1 + 1
        inc1 = 0
        while (inc1 < 3):
            if (inc1 == 0):
                mission_dict[inc1] = mission_lat
            if (inc1 == 1):
                mission_dict[inc1] = mission_lon
            if (inc1 == 2):
                mission_dict[inc1] = mission_alt
            inc1 = inc1 + 1

                        # assembles waypoints
        try:
            gotoGuardian3.targetLocation = dk.LocationGlobalRelative(mission_dict[0][self.wpNum],mission_dict[1][self.wpNum],mission_dict[2][self.wpNum])
        except IndexError:
            if gotoGuardian3.doingMission == 1:
                # only set to RTL once in case we want to manually take over after the iniaial RTL command
                self.vehicle.mode = VehicleMode("RTL") # RTL if we're out of waypoints. Kind of nasty but it'll work
                print "No more waypoints, RTL"
                self.runInterval = 5
                gotoGuardian3.doingMission = 0

    def doMission(self):
        # before sending commands to go somewhere, check if we can do mission
        # for example, if we have been told to RTL by the pilot, don't try to go anywhere
        # also check if we're in a pause condition
        if gotoGuardian3.doingMission == 0:
            print self.vehicle.mode
            return 0
        elif (self.vehicle.mode == "GUIDED"):
            gotoGuardian3.doingMission = 1 # we're good to send a command
            return 1
        else:
            gotoGuardian3.doingMission = 0
            print self.vehicle.mode
            return 0

    def checkIfPointReached(self,targetLocation):
        wpDistance = dk.get_distance_metres(self.vehicle.location.global_relative_frame, gotoGuardian3.targetLocation)
        # if(wpDistance <= targetDistance*0.01 or wpDistance < 0.3):
        #     return 1
        # else:
        #     return 0
        if(wpDistance < waypointReachedDistance):
            return 1
        else:
            return 0

    def go(self):
        # this function calculates the velocity vectors and does the logic to move towards waypoints.
        # target is the current waypoint

        targetLocation = gotoGuardian3.targetLocation
        currentLocation = self.vehicle.location.global_relative_frame
        targetDistance = dk.get_distance_metres(currentLocation, targetLocation)
        print "Going to waypoint: ", (self.wpNum+1) # human format (1 indexed)

        if self.wpNum == dropWaypointNum and self.dropped == 0:
            self.dropped = 1
            self.pause = 20 #wait 5 seconds
            drop_guided(self.vehicle)

        if (targetDistance > 20):
            if not checkObstacle.obstacleFound: # we good
                print "Velocity control"
                bearing = getBearingGood(self.vehicle.location.global_relative_frame,targetLocation)
            else: # perform slick obstacle avoid
                print "Velocity control - obstacle avoidance"
                obstacleBearing = deg(getBearingGood(self.vehicle.location.global_relative_frame,checkObstacle.obstacleLocation))
                if checkObstacle.obstacleLocation.lat >= targetLocation.lat:
                    bearing = rad(obstacleBearing - 90)
                else:
                    bearing = rad(obstacleBearing + 90)

            # Velocity vector creation
            print "Flight bearing ", deg(bearing)
            dk.condition_yaw(self.vehicle,deg(bearing)) #point towards travel direction - doesn't work because of difference between condition_yaw expected values and the way I define a bearing

            northVelocity = defaultVelocity*cos(bearing)
            eastVelocity = defaultVelocity*sin(bearing)

            print "Velocity components: ",northVelocity,", ",eastVelocity
            print "Distance to target:", targetDistance
            print "Obstacle bearing: ", checkObstacle.obstacleBearing
            print "Obstacle is ", checkObstacle.obstacleDistance, "metres away"

            if not self.pause == 0:
                print "Pausing"
                self.pause = float(self.pause - self.runInterval)
                print self.pause

                if self.pause < 0:
                    self.pause = 0
            else: # no pause
                dk.send_ned_velocity(self.vehicle,northVelocity,eastVelocity,0) # sends the velocity components to the copter
            return 0

        elif(targetDistance <= targetGuidanceThreshold): # if close to target, go right there
            print "Going straight to target"
            print "Distance to target: ", targetDistance
            self.vehicle.simple_goto(targetLocation)
            return 0

class checkObstacle(mthread.MicroThread):
    obstacleFound = 0
    obstacleLocation = LocationGlobalRelative(obstacleLat, obstacleLon, obstacleAlt) # pull from userDefines
    obstacleBearing = 0
    obstacleDistance = 0
    def __init__(self, number, vehicleIn):
        self.num = number
        self.lastRunTime =  dt.datetime.now()
        self.interceptBearing = 0 # bearing to obstacle at intercept radius
        self.runInterval = float(1)/checkObstacleFrequency # will run every runInterval seconds (or longer if something blocks) - allows dynamic reschedule
        self.avoidRadius = obstacle_avoidRadius
        self.vehicle = vehicleIn

    def step(self):
        stepTimer = secondsSince(self.lastRunTime)
        if (stepTimer >= self.runInterval):

            currentLocation = self.vehicle.location.global_relative_frame
            checkObstacle.obstacleDistance = dk.get_distance_metres(currentLocation, checkObstacle.obstacleLocation)
            checkObstacle.obstacleBearing = deg(getBearingGood(currentLocation, checkObstacle.obstacleLocation))
            self.obstacleTargetDistance = dk.get_distance_metres(checkObstacle.obstacleLocation, gotoGuardian3.targetLocation)

            if self.canAvoid(): # only avoid obstacles if appropriate
                if (checkObstacle.obstacleDistance <= self.avoidRadius):
                    if self.interceptBearing == 0: # if it's the first encounter, record the intercept bearing
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
        if self.vehicle.mode == "RTL":
            return 0
        elif  self.obstacleTargetDistance <= self.avoidRadius:
            print "Obstacle is close to waypoint. Proceeding to waypoint. Collision possible."
            print "Obstacle to waypoint distance:", self.obstacleTargetDistance
            return 0
        else:
            return 1
