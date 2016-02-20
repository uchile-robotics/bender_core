#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

source "$BENDER_CONFIG"/setup.bash ;

# Useful Variables
pkg_name="bender_sensors";
install_path=$(rospack find $pkg_name)/install;
#install_files="$install_path"/files;
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - - - Install cameras  - - - - - - - - - - -
. "$install_path"/cameras.sh
. "$install_path"/lasers.sh
. "$install_path"/rgbd.sh



#  - - - - - - - - - Build Sensors - - - - - - - - - - 
echo -e "\n$installer Building bender_sensors";
cd "$BENDER_WORKSPACE"/..
catkin_make --only-pkg-with-deps bender_sensors

# :)
