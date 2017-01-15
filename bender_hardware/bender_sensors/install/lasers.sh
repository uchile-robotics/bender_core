#!/bin/bash


#  - - - - - - - - - Install Rules - - - - - - - - - - 
echo -e "\n$installer Installing udev rules for the hokuyo laser range finders";

# prepare folder for udev rules
bender_cd forks
sudo mkdir -p /opt/bender/udev
sudo cp ../devel/lib/urg_node/getID /opt/bender/udev/

bender_cd bender_sensors
sudo cp -f install/files/bender_hokuyo.rules /etc/udev/rules.d/bender_hokuyo.rules;
sudo udevadm control --reload

