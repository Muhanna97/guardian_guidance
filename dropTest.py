from dronekit import Vehicle
import connectToDrone
import vehicleFunctions as dk
import guardian
import mthread
import scheduler

mydrone = connectToDrone.makeConnection()
print mydrone
print mydrone.mode

# dk.arm_only(mydrone)
guardian.drop_guided(mydrone)
# guardian.drop_auto(mydrone)
