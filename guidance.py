from dronekit import Vehicle
import connectToDrone
import vehicleFunctions as dk
import guardian
import mthread
import scheduler

mydrone = connectToDrone.makeConnection()
print mydrone
print mydrone.mode

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
