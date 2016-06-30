#!/bin/bash
mkdir -p export
rosrun subscriber_android_sensors subscriber_android_sensors_node \
	_prefix:=export/SENSORS_ \
	_binary:=false \
	_sensors_topic:="/android/imu"
