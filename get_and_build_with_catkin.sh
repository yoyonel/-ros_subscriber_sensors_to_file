#!/bin/bash

mkdir -p catkin_src; cd catkin_src

##################################
# get the source from GIT remote #
##################################
if [ -z ${GIT_REMOTE_FOR_ROS_DRIVER+x} ]; then
	#GIT_REMOTE_FOR_ROS_DRIVER=https://github.com/ros-drivers/velodyne.git
	#GIT_REMOTE_FOR_ROS_DRIVER=ssh://git@gitlab.dockerforge.ign.fr:10022/li3ds/ros_velodyne.git
	
	catkin_create_pkg subscriber_android_sensors rospy roscpp android_core
	
	ln -sfn  $(realpath ../src/subscriber_android_sensors.cpp) subscriber_android_sensors/src
	ln -sfn $(realpath ../src/CMakeLists.txt) subscriber_android_sensors/.
fi

#git clone $GIT_REMOTE_FOR_ROS_DRIVER

##################################
# 
##################################
#source /opt/ros/indigo/setup.bash
echo "don't forget to source setup.bash from android_core!"

rosdep install --from-paths . --ignore-src --rosdistro indigo

mkdir -p ../catkin_ws/src && pushd ../catkin_ws

catkin_init_workspace src

ln -s `eval echo $(dirs +1)` src/

catkin_make
catkin_make install

cd ..
