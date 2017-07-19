from math import cos, sin, atan2, tan, degrees as deg, radians as rad, fabs
import vehicleFunctions as dk

def getBearingGood(currentLocation, targetLocation):
    # returns in radians!
    lat1 = rad(currentLocation['latitude'])
    lat2 = rad(targetLocation['latitude'])
    long1 = rad(currentLocation['longitude'])
    long2 = rad(targetLocation['longitude'])
    deltalong = long2-long1
    return atan2(sin(deltalong)*cos(lat2),cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltalong))

def getDistanceMetres(aLocation1, aLocation2):
    dlat = aLocation2['latitude'] - aLocation1['latitude']
    dlong = aLocation2['longitude'] - aLocation1['longitude']
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def m2geo(metres):
    return (metres/11.1)*0.0001; # convert meters to decimal degrees https://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude

def geo2m(geo):
    return (geo / 0.0001) * 11.1; #convert decimal degrees to meters

def locationFromBearing(lat, long, bearing, distanceOut):

    #http://www.movable-type.co.uk/scripts/latlong.html#destPoint
    bearing = rad(bearing)

    R = 6378137.0

    latitude = asin(sin(rad(lat))*cos(distanceOut/R) + cos(rad(lat))*sin(distanceOut/R)*cos(bearing))
    longitude = rad(long) + atan2(sin(bearing)*sin(distanceOut/R)*cos(rad(lat)),cos(distanceOut/R)-sin(rad(lat))*sin(latitude))

    return {'lat': deg(latitude), 'long': deg(longitude)}

def searchPattern(pt1, pt2, pt3, pt4, searchAltitude,  trackWidth):

    numSteps = max(fabs((pt2['latitude'] - pt3['latitude']) / m2geo(trackWidth)), fabs((pt1['latitude'] - pt4['latitude'])))
    numSteps = round(numSteps)  # next highest integer
    print "numSteps"
    lats = []
    longs = []

    bearingWest = getBearingGood(pt2, pt3)
    bearingEast = getBearingGood(pt1, pt4)

    lats.append(pt1['latitude'])
    longs.append(pt1['longitude'])
    lats.append(pt2['latitude'])
    longs.append(pt2['longitude'])

    westStep = geo2m(fabs((pt2['latitude'] - pt3['latitude'])) / numSteps)  # step size in geo units
    eastStep = geo2m(fabs((pt1['latitude'] - pt4['latitude'])) / numSteps)

    prevW = pt2
    prevE = pt1

    for i in range(numSteps):
        newpoint = locationFromBearing(prevW['latitude'], prevW['longitude'], bearingWest, westStep)
        lats.append(newpoint['latitude'])
        longs.append(newpoint['longitude'])
        prevW = newpoint

        newpoint = locationFromBearing(prevE['latitude'], prevE['longitude'], bearingEast, eastStep)
        longs.append(newpoint['longitude'])
        prevE = newpoint

    return {'lats': lats,'longs':longs,'alts':alts}


