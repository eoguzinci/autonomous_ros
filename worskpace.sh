#!/bin/sh

apt-get update
pip install -r requirements.txt
apt-get install ros-kinetic-dbw-mkz
pip install --upgrade catkin_pkg_modules
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
