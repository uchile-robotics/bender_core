#!/bin/bash

# Create URDF and SDF files from xacro
# run: $ bash bender_description/scripts/update_models.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[bender_description]:${reset}"

#  - - - - - - - - - Main - - - - - - - - - - - 
BASEDIR=$(pwd)
cd "$BENDER_WS"/base_ws/src/bender_common/bender_description/robots
echo "$installer Deleting old models."
rm -rf bender.urdf
rm -rf bender.sdf
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error deleting files.${reset}"
    exit 1 # Terminate and indicate error
fi
echo "$installer Exporting URDF."
rosrun xacro xacro.py bender_asus.urdf.xacro prefix:="bender/" > bender.urdf
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error exporting URDF.${reset}"
    exit 1 # Terminate and indicate error
fi
echo "$installer Exporting SDF."

# Check for gz command (Gazebo 3 or higher)
if [ -x "$(command -v gz)" ]; then
	gz sdf -p bender.urdf > bender.sdf
elif [ -x "$(command -v gzsdf)" ]; then
	# Check for gzsdf command (Gazebo 2)
	gzsdf print bender.urdf > bender.sdf
else
	echo "$installer ${red}Gazebo is not installed.${reset}"
fi
# Check error code
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error exporting SDF.${reset}"
    exit 1 # Terminate and indicate error
fi
cd $BASEDIR
