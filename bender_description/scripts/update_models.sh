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
cd "$UCHILE_WS"/pkgs/base_ws/bender_core/bender_description/robots

echo "$installer Exporting URDF."
for i in $(ls *.urdf.xacro);do
    urdf_name=${i%$'.urdf.xacro'}
    printf "%s\tExporting %s.urdf" "${installer}" "${urdf_name}"
    rosrun xacro xacro.py ${i} prefix:="bender/" > ${urdf_name}.urdf
    if [ $? -ne 0 ]; then
        printf "\n%s%s Error exporting %s.urdf%s\n" "${installer}" "${red}" "${urdf_name}" "${reset}"
        cd $BASEDIR
        exit 1 # Terminate and indicate error
    else
        printf " %s\xe2\x9c\x93\n%s" "${green}" "${reset}"
    fi
done


echo "$installer Exporting SDF."
mkdir -p $HOME/.gazebo # prevent log errors
for i in $(ls *.urdf);do
    urdf_name=${i%$'.urdf'}
    printf "%s\tExporting %s.sdf" "${installer}" "${urdf_name}"
    # Check for gz command (Gazebo 3 or higher)
    if [ -x "$(command -v gz)" ]; then
        gz sdf -p ${urdf_name}.urdf > ${urdf_name}.sdf
    elif [ -x "$(command -v gzsdf)" ]; then
        # Check for gzsdf command (Gazebo 2)
        gzsdf print ${urdf_name}.urdf > ${urdf_name}.sdf
    else
        echo "$installer ${red}Gazebo is not installed.${reset}"
        cd $BASEDIR
        exit 1
    fi
    if [ $? -ne 0 ]; then
        printf "\n%s%s Error exporting %s.sdf%s\n" "${installer}" "${red}" "${urdf_name}" "${reset}"
        cd $BASEDIR
        exit 1 # Terminate and indicate error
    else
        printf " %s\xe2\x9c\x93\n%s" "${green}" "${reset}"
    fi
done

cd $BASEDIR
