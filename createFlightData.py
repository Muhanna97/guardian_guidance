from userDefines import * # waypoints, default velocity
import emergentObject2 as emergent2

def createFlightData():
    if usingInterop:
        baseLats = missionDict['wp_lats']
        baseLongs = missionDict['wp_longs']
        baseAlts = missionDict['wp_alts']

        print "BASE LATITUDES: ", baseLats

        # insert the drop location as a waypoint according to drop location pt (defined in mission), and drop altitude
        # (defined in userDefines)
        baseLats.insert(dropWaypointNum-1, missionDict['drop_lat'])
        baseLongs.insert(dropWaypointNum-1, missionDict['drop_long'])
        baseAlts.insert(dropWaypointNum-1, dropHeight)

        # deal with the emergent obstacle generated wps
        emergentFirstWP = len(baseLats) + 1
        # emergent_waypoints = emergent.calcEmergent(missionDict['emergent_lat'], missionDict['emergent_long'], emergent_searchAltitude, emergent_searchRadius)

        baseLats.append(missionDict['emergent_lat'])
        baseLongs.append(missionDict['emergent_long'])
        baseAlts.append(emergent_searchAltitude)


        emergent_waypoints = emergent.calcEmergentSearch(missionDict['emergent_lat'], missionDict['emergent_long'], emergent_searchAltitude, emergent_searchRadius,emergent_numSpirals)
        baseLats.extend(emergent_waypoints['lats'])
        baseLongs.extend(emergent_waypoints['longs'])
        baseAlts.extend(emergent_waypoints['alts'])

        return {'lats': baseLats, 'longs': baseLongs, 'alts': baseAlts, 'emergentFirstWp': emergentFirstWP}

    else:  # using built in values
        emergentFirstWP = len(lats) + 1
        emergent_waypoints = emergent2.calcEmergentSearch(emergentLat, emergentLong, emergent_searchAltitude, emergent_searchRadius, emergent_numSpirals)

        lats.extend(emergent_waypoints['lats'])
        longs.extend(emergent_waypoints['longs'])
        alts.extend(emergent_waypoints['alts'])

        return {'lats': lats, 'longs': longs, 'alts': alts, 'emergentFirstWp': emergentFirstWP}
