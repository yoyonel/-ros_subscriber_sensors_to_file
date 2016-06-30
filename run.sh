#!/bin/bash
mkdir -p export
rosrun subscriber_android_sensors subscriber_android_sensors_node _prefix:=export/SENSORS_
