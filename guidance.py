from dronekit import Vehicle
import connectToDrone
import vehicleFunctions as dk
import guardian
import mthread
import scheduler
import userDefines
import importInterop

mydrone = connectToDrone.makeConnection()
print mydrone
print mydrone.mode


# print "\nPrint all parameters (iterate `vehicle.parameters`):"
# for key, value in mydrone.parameters.iteritems():
#     print " Key:%s Value:%s" % (key,value)

# print userDefines.missionDict

# importInterop.importFromInterop()

@mydrone.parameters.on_attribute(userDefines.DROP_PARAM)
def decorated_thr_min_callback(self, attr_name, value):
    print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

guardian.close_Dropper(mydrone)
dk.arm_and_takeoff(mydrone, 30)

print "Team Guardian AUVSI Guidance Script"

# create cooperatively scheduled threads
mt1 = guardian.gotoGuardian3(1, mydrone)
mt2 = guardian.checkObstacle(2, mydrone)


ms = scheduler.Scheduler()
ms.add_microthread(mt1)
ms.add_microthread(mt2)

# run the threads - currently no exit from this, so LAND and CLOSE section will not be reached
# ctrl+c the script to kill it
ms.run()
