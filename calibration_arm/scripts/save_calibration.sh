#!/bin/bash

# obtiene los parametros de dynamic_tf y los guarda en calibation.yaml con la fecha del dia donde se realiza
# run: $ bash calibration_arm/scripts/save_calibration.sh

#---------Setup--------------
calibration_folder="$BENDER_WS/base_ws/src/calibration_arm/calibation"
fecha="$(date +'%s')"

#-Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

#----------Main--------------
#export LAST_DATE_CALIBRATION="$fecha"
echo "${green}$fecha.${reset}"
rosparam get /bender/dynamic_tf/ > bender_description/calibration/calibration_"$fecha".yaml
echo "${green}Se guardo con la siguente fecha formato= epoch $fecha${reset}"