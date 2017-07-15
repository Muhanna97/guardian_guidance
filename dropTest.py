from dronekit import Vehicle
import connectToDrone
import vehicleFunctions as dk
import guardian
import userDefines


mydrone = connectToDrone.makeConnection()
print mydrone
print mydrone.mode

@mydrone.parameters.on_attribute(userDefines.DROP_PARAM)
def decorated_thr_min_callback(self, attr_name, value):
    print " PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)

guardian.close_Dropper(mydrone)
dk.arm_only(mydrone)
guardian.drop_guided(mydrone)
