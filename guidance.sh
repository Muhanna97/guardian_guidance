#!/bin/sh

cd ~/ardupilot/Tools/autotest

# Open up a new tab
# Thanks stack overflow! https://stackoverflow.com/questions/1188959/open-a-new-tab-in-gnome-terminal-using-command-line
WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')
xdotool windowfocus $WID
xdotool key ctrl+shift+t
wmctrl -i -a $WID

python sim_vehicle.py -w -v ArduCopter --console --map --location SRY --out 127.0.0.1:14540 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14549 --out 127.0.0.1:14541 --out 127.0.0.1:14552


