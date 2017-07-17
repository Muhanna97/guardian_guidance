---------------- Do once -------------------
1. Install APM SITL: http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
2. Install DroneKit http://python.dronekit.io/develop/installation.html
3. Clone this repo


4. Add the surrey field to locations.txt in ardupilot/Tools/autotest with the following line:
    SRY=49.128645,-122.7965086,100,0

5. You should have the following: 
    username/dronekit-python
    username/guardian_guidance
    username/ardupilot

-------------------- Running the Script --------------------
1. In a terminal window, cd to ardupilot/Tools/autotest

2. run:
    python sim_vehicle.py -w -v ArduCopter --console --map --location SRY

3. Open a new window, cd to guardian_guidance

4. run:
    python guidance.py

5. Currently, the script doesn't exit cleanly
    ctrl+c to kill it :D

-------------------------- How it works ------------------
The dronekit api lets creates MAVLink commands to control or get information from the vehicle.
MAVProxy takes the MAVLink stream to/from the vehicle and lets us connect to it with Dronekit and APM planner.

The script cooperatively schedules two tasks - checking for obstacles, and sending vehicle commands.
To fly towards a waypoint, a bearing is calculated between the drone and the waypoint.
Then, the North and East velocity components of the vehicle are changed to head along this bearing.
When the drone is close to the object, a goto command is sent. This uses the standard waypoint navigation control system, so automatically slows the drone down as it approaches the waypoint
This prevents us from having to make our own control system.

If the drone is close to an obstacle, the velocity vector is changed to curve around the obstacle.
When it has passed and avoided the obstacle, it continues to head towards the current waypoint.

When all waypoints have been reached, the vehicle will RTL.

There is lots of logic to check that we're in the right mode.
The script will only send velocity commands to the drone if it is in GUIDED mode, which the script places it in by default.
This means that we can switch the drone to RTL, STABILIZE, etc. at any time (using the RC controller or APM planner), to stop the script from having control. This prevents bad things from happening.


---------------------- Relevant sections -------------------
guidance.py creates the tasks, connects and takes off
the tasks are for guidance and obstacle proximity checking, and are defined in guardian.py
userDefines.py has settable params that might need to be changed to tune perfomrace


The step method in the checkObstacle and gotoGuardian3 classes are run repeatedly by the scheduler, according to the scheduled frequency of the task

---------------------- Running a flight -----------------------
connect to the stream:
    mavproxy.py --out=127.0.0.1:14550 --out=127.0.0.1:14551 --out=127.0.0.1:14540 --out=127.0.0.1:14549 --out=127.0.0.1:14541^C

with that, connect to tracker in guardian directory
    sudo python antenna_data.py

can start up apm planner for monitoring

connect to interop somehow (will die once you connect to guidance :()

take off to 30m in loiter. Start guidance script
