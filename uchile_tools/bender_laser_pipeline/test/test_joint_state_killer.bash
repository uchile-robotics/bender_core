#!/bin/bash

# we need to kill this node when running a rosbag to avoid weird jumps
# on the robot joints, as they are already published by the rosbag.
NODE="/joint_state_publisher"
while true; do
	sleep 1
	if [ ! "$(rosnode list | grep "$NODE" | wc -c)" = "0" ]; then
		echo "Killing node $NODE ..."
		rosnode kill "$NODE"
		echo "Killing node $NODE ... KILLED"
		exit 0
	fi
done