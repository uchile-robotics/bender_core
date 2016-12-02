#!/bin/bash

# obtiene los parametros de dynamic_tf y los guarda en calibation.yaml con la fecha del dia donde se realiza
# run: $ bash calibration_arm/scripts/save_calibration.sh

#---------Setup--------------
calibration_folder="$BENDER_WS/base_ws/src/calibration_arm/calibration"
fecha="$(date +'%s')"
#-Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

trap ctrl_c INT


function ctrl_c()
{
	rosparam get /bender/dynamic_tf/ > "$calibration_folder"/calibration_"$fecha".yaml
	echo "${green}Se guardo con la siguente fecha formato= epoch $fecha${reset}"
	exit
}

while [ 1 ]
do
    sleep 0.1
done

#----------Main--------------
