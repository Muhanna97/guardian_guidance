#!/bin/bash
# This script installs APM, DroneKit, and required dependencies.
# Running this will get you the correct environment and configuration to work on the guidance script.
# Author: Richard Arthurs
# Written: Oct. 4, 2017

echo "--------------------- Team Guardian Autopilot Setup Script!!! :D ---------------------"
echo "Note: you will need to enter your password and 'y' several times."
echo "Installation takes roughly 10 minutes."
echo "Please modify this script if you don't want things in your root directory. (see comments)"
echo ""

echo "--------------------- Cloning APM from git ---------------------"
echo ""
cd ~/ #Modify for non-root install
git clone git://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot #Modify for non-root install
git submodule update --init --recursive

sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
sudo apt-get install python-scipy python-opencv ccache gawk git python-pip python-pexpect
sudo pip install future pymavlink MAVProxy

export PATH=$PATH:$HOME/jsbsim/src
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH

. ~/.bashrc

cd ~/ #Modify for non-root install

echo "--------------------- Installing DroneKit ---------------------"
echo ""
sudo apt-get install python-pip python-dev
pip install dronekit

echo "--------------------- Updating APM locations.txt to include Surrey field ---------------------" # we need this to avoid typing in coordinates every time, or flying from Australia to Surrey
echo ""
cp ~/guardian_guidance/locations.txt ~/ardupilot/Tools/autotest #Modify for non-root install

echo "--------------------- Installing other dependencies ---------------------"
echo ""
sudo apt-get install xdotool

echo "--------------------- Good to go!---------------------"
echo ""
echo ""
echo "/------------------------\              |~~\_____/~~\__  |
|                        |______________ \______====== )-+
|     TEAM GUARDIAN      |                      ~~~|/~~  |
\------------------------/                         ()
"
