#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

installer="[INSTALLER]:"
bender_cd bender_sensors


#  - - - - - - - - - Install Rules - - - - - - - - - - 
echo -e "\n$installer Installing udev rules for the hokuyo laser range finders";
sudo cp -f install/files/bender_hokuyo.rules /etc/udev/rules.d/bender_hokuyo.rules;
sudo udevadm control --reload
