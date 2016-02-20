#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

source "$BENDER_CONFIG"/setup.bash ;

# Useful Variables
pkg_name="bender_sensors";
install_path=$(rospack find $pkg_name)/install;
install_files="$install_path"/files;
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - - - ROS packages  - - - - - - - - - - -
echo -e "\n$installer Installing laser packages . . ."

# Laser Drivers (Hokuyo)
sudo apt-get install ros-indigo-hokuyo-node ;


#  - - - - - - - - - Install Rules - - - - - - - - - - 
echo -e "\n$installer Installing udev rules for the hokuyo laser range finders";
sudo cp -f "$install_files"/bender_hokuyo.rules /etc/udev/rules.d/bender_hokuyo.rules;
sudo udevadm control --reload
