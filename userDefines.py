# all of the user settabble params for guidance
# Author: Richard Arthurs



defaultVelocity = 5 # this velocity is split into North and East components to control direction of copter

# Waypoint data - will loop through all of these
lats = [49.128695,49.1294287000000]
longs = [-122.800132, -122.796303000000]
alts = [30,30];

# Obstacle data - currently no support for multiple obstacles, but it could be implemented
obstacleLat = 49.128400
obstacleLon = -122.798044
obstacleAlt = 30

# distance thresholds
targetGuidanceThreshold = 20 # at this distance (m) or below, vehicle will use simple_goto to reach waypoint instead of custom velocity control
obstacle_avoidRadius = 50 # if we are within this many metres from obstacle point, we will avoid it
waypointReachedDistance = 1 # if we are within this many metres of waypoint, consider it reached (typically 2m in actual operation)

#Task Frequencies
# tasks will run this many times per second, or slower (if something is blocking, such as using a time delay command - wHich we WON'T DO!)
checkObstacleFrequency = 10
guidanceFrequency = 10 # rate (Hz) that velocity components are sent to vehicle
