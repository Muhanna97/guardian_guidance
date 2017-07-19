from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Vehicle, Command
from pymavlink import mavutil # Needed for command message definitions
import time
import datetime as dt
import mthread
from math import sin, cos, atan2, asin, atan
from math import radians as rad
from math import degrees as deg
import vehicleFunctions as dk
from userDefines import * # waypoints, default velocity
import emergentObject as emergent
import numpy

# Functions and classes written specifically for guardian obstacle avoidance
# Author: Richard Arthurs
def close_Dropper(vehicle):
    print "Closing dropper"
    vehicle.parameters[DROP_PARAM] = 57

def drop_guided(vehicle):
# sends a command to drop the bottle in guided mode.
    print "Dropping bottle!"

    runs = 0 # so we can trigger the servo multiple times

    while runs < dropTriggerRepeats:
        vehicle.parameters[DROP_PARAM] = 0 # set parameter allowing us to change the servo value from DroneKit
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            servoNum,    # servo number (userDefines)
            servoOpen,          # servo open PWM value
            0, 0, 0, 0, 0)    # not used

        vehicle.send_mavlink(msg)
        time.sleep(1)
        close_Dropper(vehicle)
        runs += 1

def getBearingGood(currentLocation, targetLocation):
    # returns in radians!
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

def createFlightData():
    if usingInterop:
        baseLats = missionDict['wp_lats']
        baseLongs = missionDict['wp_longs']
        baseAlts = missionDict['wp_alts']

        # insert the drop location as a waypoint according to drop location pt (defined in mission), and drop altitude
        # (defined in userDefines)
        baseLats.insert(dropWaypointNum-1, missionDict['drop_lat'])
        baseLongs.insert(dropWaypointNum-1, missionDict['drop_long'])
        baseAlts.insert(dropWaypointNum-1, dropHeight)

        # deal with the emergent obstacle generated wps
        emergentFirstWP = len(baseLats) + 1
        emergent_waypoints = emergent.calcEmergent(missionDict['emergent_lat'], missionDict['emergent_long'], emergent_searchAltitude, emergent_searchRadius)

        baseLats.extend(emergent_waypoints['lats'])
        baseLongs.extend(emergent_waypoints['longs'])
        baseAlts.extend(emergent_waypoints['alts'])

        return {'lats': baseLats, 'longs': baseLongs, 'alts': baseAlts, 'emergentFirstWp': emergentFirstWP}

    else:  # using built in values
        emergentFirstWP = len(lats) + 1
        emergent_waypoints = emergent.calcEmergent(emergentLat, emergentLong, emergent_searchAltitude, emergent_searchRadius)

        lats.extend(emergent_waypoints['lats'])
        longs.extend(emergent_waypoints['longs'])
        alts.extend(emergent_waypoints['alts'])

        return {'lats': lats, 'longs': longs, 'alts': alts, 'emergentFirstWp': emergentFirstWP}


def setObstacleLocation():
    if usingInterop:
        return LocationGlobalRelative(missionDict['obstacle_lat'], missionDict['obstacle_long'], 30)
    else:
        return LocationGlobalRelative(obstacleLat, obstacleLon, obstacleAlt) # pull from userDefines

def setAvoidRadius():
    if usingInterop:
        return obstacle_avoidRadius + (missionDict['obstacle_radius'] / 3)
    else:
      return obstacle_avoidRadius

def locationAlongBearing(from_location, to_location, distanceOut):
    #http://www.movable-type.co.uk/scripts/latlong.html#destPoint
    bearing = getBearingGood(from_location, to_location)
    print "degrees", bearing*180/3.14159
    print from_location
    print to_location
    R = 6378137.0

    latitude = asin(sin(rad(from_location.lat))*cos(distanceOut/R) + cos(rad(from_location.lat))*sin(distanceOut/R)*cos(bearing))
    longitude = rad(from_location.lon) + atan2(sin(bearing)*sin(distanceOut/R)*cos(rad(from_location.lat)),cos(distanceOut/R)-sin(rad(from_location.lat))*sin(latitude))

    return LocationGlobalRelative(deg(latitude), deg(longitude), from_location.alt)

def distance_to_intercept(drone_location, obstacle_location, avoid_radius, flight_bearing):
    # uses the law of cosines to determine a radius out to place the waypoint
    # along an intercept radius from the obstacle

    b = dk.get_distance_metres(drone_location, obstacle_location)
    c = avoid_radius
    obstacle_bearing = getBearingGood(drone_location, obstacle_location)

    c_rad = obstacle_bearing - flight_bearing

    term1 = -2*b*cos(c_rad)
    term0 = -(c**2 - b**2)

    print "obs dist:", b, "avoid radius:", c, "obs bearing: ", deg(obstacle_bearing), "flight bearing: ", deg(flight_bearing)
    print term1, term0
    distances = numpy.roots([1, term1, term0])
    print min(distances)
    print "Distance results::::::::::::::::::::", distances
    return distances

class gotoGuardian3(mthread.MicroThread):
    targetLocation = LocationGlobalRelative(49.130629, -122.793948, 100) # Initializing, will be changed later
    doingMission = 0

    def __init__(self, number, vehicleIn):
        self.num = number  # for scheduler
        self.runInterval = float(1)/guidanceFrequency # will run every runInterval seconds (or longer if something blocks) - allows dynamic reschedule
        self.wpNum = startWpNum  # current wp we're flying to
        self.lastRunTime = dt.datetime.now()
        self.vehicle = vehicleIn
        self.dropped = 0

        flight_data = createFlightData()
        self.lats = flight_data['lats']
        self.longs = flight_data['longs']
        self.alts = flight_data['alts']
        self.emergentFirstWP = flight_data['emergentFirstWp']
        self.encountered = 0

    def step(self):
        stepTimer = secondsSince(self.lastRunTime)
        if (stepTimer >= self.runInterval):
            print  "\r\n"

            self.createwp()  # create the current target wp
            if self.doMission():  # if we can do mission (not in RTL or some other case)
                self.go()  # go to the target location

            if self.checkIfPointReached():
                self.wpNum += 1

            self.lastRunTime = dt.datetime.now()

    def createwp(self):
        # assembles waypoints
        try:
                # gotoGuardian3.targetLocation = dk.LocationGlobalRelative(lats[self.wpNum], longs[self.wpNum], alts[self.wpNum])
                gotoGuardian3.targetLocation = dk.LocationGlobalRelative(self.lats[self.wpNum], self.longs[self.wpNum], self.alts[self.wpNum])
        except IndexError:
            if gotoGuardian3.doingMission == 1:
                # only set to RTL once in case we want to manually take over after the iniaial RTL command
                self.vehicle.mode = VehicleMode("RTL") # RTL if we're out of waypoints. Kind of nasty but it'll work
                print "No more waypoints, RTL"
                time.sleep(2)  # give it time to send the RTL

            self.runInterval = 5 # we can run this less frequently now that we're just RTL'ing
            gotoGuardian3.doingMission = 0

    def doMission(self):
        # before sending commands to go somewhere, check if we can do mission
        # for example, if we have been told to RTL by the pilot, don't try to go anywhere

        if (self.vehicle.mode == "GUIDED"):
            gotoGuardian3.doingMission = 1  # we're good to send a command
            return 1
        elif gotoGuardian3.doingMission == 0:
            print self.vehicle.mode
            return 0
        else:  # basically if we are in anything but guided, don't send movement commands
            gotoGuardian3.doingMission = 0
            print self.vehicle.mode
            return 0

    def checkIfPointReached(self):
        wpDistance = dk.get_distance_metres(self.vehicle.location.global_relative_frame, gotoGuardian3.targetLocation)

        if(wpDistance < waypointReachedDistance):
            if self.doMission():
                print "Reached waypoint, pausing..."
                self.stop(2)

                # bottle drop logic
                if self.wpNum == dropWaypointNum-1 and self.dropped == 0:
                    self.dropped = 1
                    print "Reached drop location, hovering.."
                    self.stop(2)
                    drop_guided(self.vehicle)
                    self.stop(2)

                print "Resuming flight"

                if self.emergentFirstWP == self.wpNum -1:
                    print "Beginning emergent object search"
                    self.stop(2)
                return 1
        else:
            return 0

    def stop(self, sleepTime): # hover for a bit
        if self.doMission():
            dk.send_ned_velocity(self.vehicle, 0, 0,0)  # sends the velocity components to the copter
            print "Hovering...."
            time.sleep(sleepTime)
        else:
            print "Not able to do mission."


    def go(self):
        # this function calculates the velocity vectors and does the logic to move towards waypoints.
        # target is the current waypoint

        targetLocation = gotoGuardian3.targetLocation
        currentLocation = self.vehicle.location.global_relative_frame
        targetDistance = dk.get_distance_metres(currentLocation, targetLocation)
        print "Going to waypoint: ", (self.wpNum+1)  # human format (1 indexed)


        if (targetDistance > targetGuidanceThreshold):
            if not checkObstacle.obstacleFound: # we good
                print "Velocity control"
                bearing = getBearingGood(self.vehicle.location.global_relative_frame,targetLocation)

                # Velocity vector creation
                print "Distance to target:", targetDistance
                print "     Flight bearing ", deg(bearing)
                dk.condition_yaw(self.vehicle,deg(bearing))

                northVelocity = defaultVelocity*cos(bearing)
                eastVelocity = defaultVelocity*sin(bearing)

                print "     Velocity components: ",northVelocity,", ",eastVelocity
                print "     Obstacle bearing: ", checkObstacle.obstacleBearing
                print "     Obstacle is ", checkObstacle.obstacleDistance, "metres away"

                dk.send_ned_velocity(self.vehicle,northVelocity,eastVelocity,0) # sends the velocity components to the copter

            else: #obstacle
                # fly to intercept point on buffer radius from obstacle
                if self.encountered == 0: #first time being by obstacle
                    interceptLocation = locationAlongBearing(self.vehicle.location.global_relative_frame, targetLocation, min(checkObstacle.intercept_out))
                    exitLocation = locationAlongBearing(self.vehicle.location.global_relative_frame, targetLocation, max(checkObstacle.intercept_out))
                    print "Distance to intercept waypoint: ", dk.get_distance_metres(self.vehicle.location.global_relative_frame, interceptLocation)

                    self.vehicle.simple_goto(interceptLocation)
                    self.encountered = 1

                    while dk.get_distance_metres(self.vehicle.location.global_relative_frame, interceptLocation) > waypointReachedDistance:
                        print "Distance to intercept waypoint: ", dk.get_distance_metres(self.vehicle.location.global_relative_frame, interceptLocation)
                        self.vehicle.simple_goto(interceptLocation) # resend since connection sometimes drops
                        time.sleep(0.5)

                    print "Hovering at intercept waypoint!"
                    self.stop(2)

                    deriv_ExitDistance = 100
                    prev_ExitDistance = 1000

                    while dk.get_distance_metres(self.vehicle.location.global_relative_frame, exitLocation) > gotoExitDistance:
                        if deriv_ExitDistance > 0:
                            print "Velocity control - obstacle avoidance"
                            exitDistance = dk.get_distance_metres(self.vehicle.location.global_relative_frame, exitLocation)
                            print " Distance to exit waypoint:", exitDistance

                            obstacleBearing = deg(getBearingGood(self.vehicle.location.global_relative_frame,checkObstacle.obstacleLocation))
                            bearing = rad(obstacleBearing - 90)
                            northVelocity = circleSpeed * cos(bearing)
                            eastVelocity = circleSpeed * sin(bearing)
                            dk.send_ned_velocity(self.vehicle, northVelocity, eastVelocity, 0)
                            deriv_ExitDistance = prev_ExitDistance - exitDistance
                            print " Deriv:", deriv_ExitDistance
                            prev_ExitDistance = exitDistance
                            time.sleep(0.5)
                        else:
                            break

                    self.stop(2)
                    print "Going to exit waypoint!"
                    self.vehicle.simple_goto(exitLocation)

                    while dk.get_distance_metres(self.vehicle.location.global_relative_frame, exitLocation) > waypointReachedDistance:
                        print "Distance to exit waypoint: ", dk.get_distance_metres(self.vehicle.location.global_relative_frame, exitLocation)
                        self.vehicle.simple_goto(exitLocation)
                        time.sleep(0.5)

                    print "Hovering at exit waypoint!"
                    self.stop(2)

            return 0

        elif(targetDistance <= targetGuidanceThreshold): # if close to target, go right there
            print "Going straight to target"
            print "Distance to target: ", targetDistance
            self.vehicle.simple_goto(targetLocation)
            return 0


class checkObstacle(mthread.MicroThread):
    obstacleFound = 0
    obstacleBearing = 0
    obstacleDistance = 0

    obstacleLocation = setObstacleLocation()
    avoidRadius = setAvoidRadius()
    intercept_out = 0

    def __init__(self, number, vehicleIn):
        self.num = number
        self.lastRunTime =  dt.datetime.now()
        self.interceptBearing = 0 # bearing to obstacle at intercept radius
        self.runInterval = float(1)/checkObstacleFrequency # will run every runInterval seconds (or longer if something blocks) - allows dynamic reschedule
        self.vehicle = vehicleIn
        self.encountered =0

    def step(self):
        stepTimer = secondsSince(self.lastRunTime)
        if (stepTimer >= self.runInterval):

            currentLocation = self.vehicle.location.global_relative_frame
            checkObstacle.obstacleDistance = dk.get_distance_metres(currentLocation, checkObstacle.obstacleLocation)
            checkObstacle.obstacleBearing = deg(getBearingGood(currentLocation, checkObstacle.obstacleLocation))
            self.obstacleTargetDistance = dk.get_distance_metres(checkObstacle.obstacleLocation, gotoGuardian3.targetLocation)

            if self.canAvoid(): # only avoid obstacles if appropriate
                if (checkObstacle.obstacleDistance <= obstacle_gotoInterceptDistance):
                    # if self.interceptBearing == 0: # if it's the first encounter, record the intercept bearing
                    #     self.interceptBearing = checkObstacle.obstacleBearing
                    if self.encountered == 0:
                        checkObstacle.obstacleFound = 1
                        flight_bearing = getBearingGood(currentLocation, gotoGuardian3.targetLocation)
                        checkObstacle.intercept_out = distance_to_intercept(currentLocation, checkObstacle.obstacleLocation,
                                                                        checkObstacle.avoidRadius, flight_bearing)
                        self.encountered = 1

                else:
                    checkObstacle.obstacleFound = 0

                self.lastRunTime = dt.datetime.now()

    def canAvoid(self):
        # we want to skip obstacle avoidance in RTL mode (because it might be a failsafe)
        # also skip if waypoint is within obstacle avoid radius. Would rather hit obstacle than miss waypoint.
        if gotoGuardian3.doingMission == 0:
            return 0
        if not self.vehicle.mode == "GUIDED":
            return 0
        elif self.obstacleTargetDistance <= checkObstacle.avoidRadius:
            print "Obstacle is close to waypoint. Proceeding to waypoint. Collision possible."
            print "Obstacle to waypoint distance:", self.obstacleTargetDistance
            return 0
        else:
            return 1
