#!/bin/bash
#
# Run me like this
# > bash install.sh
#
# DO NOT USE ONE OF THIS:
# > source install.sh
# > . install.sh
# > ./install.sh
#

# - - - - - - S E T U P - - - - - - - -
source "$BENDER_WS"/bender_system/install/pkg_install.bash


# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[uchile_turning_base]:${reset}"

#  - - - - - - - - - Install Rules - - - - - - - - - - - 
echo "$installer Installing udev rules"
bender_cd uchile_turning_base
sudo cp -f install/10-uchile_turning_base.rules /etc/udev/rules.d/10-uchile_turning_base.rules
sudo udevadm control --reload

#  - - - - - - - - - Port Permissions  - - - - - - - - - 
# Dialout group
echo "$installer Add user to dialout group"
sudo usermod -a -G dialout "$USER"
# Port permissions
echo "$installer Add ports permissions"
if [ -e /dev/bender/turning_base ];
then
   real_port=$(readlink -f '/dev/bender/turning_base')
   echo "$installer Port /dev/bender/turning_base connected at ${real_port}"
   echo "$installer Add ports permissions"
   sudo chmod a+rw ${real_port}
else
   echo "$installer ${red}Port /dev/bender/turning_base NOT connected.${reset}"
   echo "$installer ${yellow}You must add permissions using \$ sudo chmod a+rw \$(readlink -f '/dev/bender/turning_base')${reset}"
fi

echo "$installer ${yellow}The computer must be restarted in order to complete the installation${reset}"
