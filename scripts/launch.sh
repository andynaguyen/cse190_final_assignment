#!/bin/bash

cd ~/catkin_ws
catkin_make

echo roslaunching
echo 3...
echo 2...
echo 1...

roslaunch cse_190_assi_1 solution_python.launch
