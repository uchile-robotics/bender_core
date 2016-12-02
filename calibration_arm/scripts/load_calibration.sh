#!/bin/bash

# lanza rqt para modificar offset
# run: $ bash calibration_arm/scripts/load_calibration

#---------Setup--------------
calibration_folder="$BENDER_WS/base_ws/src/calibration_arm/calibration"

# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)
# Line tittle
installer="${bold}[bender_description]:${reset}"
#----------Main--------------
last_file=$(ls -1r $calibration_folder | head -n 1)
echo "${green}$installer Loading calibration file: $calibration_folder/$last_file.${reset}"

# Se toma la calibracion ultima (pendiente) , falta revisar la fecha mas reciente.
rosparam load "$calibration_folder"/"$last_file" /bender/dynamic_tf/
