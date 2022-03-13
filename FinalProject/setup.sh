#!/bin/bash
echo "Copying gifter code to ROS workspace"
cp -R ~/RobotSystems/armProject/gifter ~/armpi_fpv/src/
echo "Copying parrot code to ROS workspace"
cp -R ~/RobotSystems/armProject/parrot ~/armpi_fpv/src/
echo "Copying launch file to armpi_fpv_bringup"
cp ~/RobotSystems/armProject/start_functions.launch ~/armpi_fpv/src/armpi_fpv_bringup/launch/
echo "Building ROS workspace"
cd ~/armpi_fpv && catkin_make
echo "Sourcing workspace"
source ~/armpi_fpv/devel/setup.bash
echo "Roslaunch test"
roslaunch /home/ubuntu/armpi_fpv/src/armpi_fpv_bringup/launch/start_functions.launch
