#!/bin/bash

echo "Cloning APM from git..."
cd ~/
git clone git://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot
git submodule update --init --recursive

sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
sudo apt-get install python-scipy python-opencv ccache gawk git python-pip python-pexpect
sudo pip install future pymavlink MAVProxy

export PATH=$PATH:$HOME/jsbsim/src
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

. ~/.bashrc

cd ~/

echo "Installing DroneKit..."
sudo apt-get install python-pip python-dev
pip install dronekit

echo "Updating APM locations.txt to include Surrey field"
cp ~/guardian_guidance/locations.txt ~/ardupilot/Tools/autotest

echo "Installing other dependencies"
sudo apt-get install xdotool

echo "Good to go!"
