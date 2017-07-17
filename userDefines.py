# all of the user settable params for guidance
# Author: Richard Arthurs
import importInterop

defaultVelocity = 7 # this velocity is split into North and East components to control direction of copter

UDPPort = "127.0.0.1:14540"

usingInterop = 0

if usingInterop:
    missionDict = importInterop.importFromInterop()

# distance thresholds
targetGuidanceThreshold = 30    # at this distance (m) or below, vehicle will use simple_goto to reach waypoint instead of custom velocity control
obstacle_avoidRadius = 15   # if we are within this many metres from obstacle point, we will avoid it
waypointReachedDistance = 2     # if we are within this many metres of waypoint, consider it reached (typically 2m in actual operation)

#Task Frequencies
# tasks will run this many times per second, or slower (if something is blocking, such as using a time delay command - wHich we WON'T DO!)
checkObstacleFrequency = 2
guidanceFrequency = 2   # rate (Hz) that velocity components are sent to vehicle

# Dropping servo params
DROP_PARAM = 'RC7_FUNCTION'  # either RC7_ or SERVO7_
servoNum = 7
servoOpen = 1200    # pwm for Dropping
dropWaypointNum = 2     # the copter will drop when it reaches this waypoint
dropHeight = 20
dropTriggerRepeats = 2 # number of times to trigger the drop. Attempt to reduce snags.


# lats = [49.128811,49.129285, 49.129127, 49.128815]
# longs = [-122.796816,-122.795733, -122.797900, -122.797214]
# ---------------------- simple mission 1 -------------------------------------------
lats = [49.128811, 49.129127, 49.128815]
longs = [-122.796816, -122.797900, -122.797214]
alts = [30,30,30]

obstacleLat = 49.130204
obstacleLon = -122.792854
obstacleAlt = 30

# -----------------------  long mission ----------------------------------------
# lats = [49.129553, 49.129507, 49.129017, 49.128874, 49.129736, 49.128963, 49.129098]
# longs = [-122.795013, -122.793344, -122.793733, -122.796524, -122.795497, -122.797562, -122.798022]
# alts = [30,30,30,30,30,30,30,30]

# obstacleLat = 49.129137
# obstacleLon = -122.796990
# obstacleAlt = 30

# ------------------------  default tester mission ------------------------
# lats = [49.128695,49.1294287000000]
# longs = [-122.800132, -122.796303000000]
# alts = [30,30];

# Obstacle data - currently no support for multiple obstacles, but it could be implemented
# obstacleLat = 49.128400
# obstacleLon = -122.798044
# obstacleAlt = 30


