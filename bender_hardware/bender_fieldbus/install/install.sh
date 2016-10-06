#!/bin/bash

# bender_fieldbus package installer
# run: $ bash bender_fieldbus/install/install.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[bender_fieldbus]:${reset}"

#  - - - - - - - - - Install Rules - - - - - - - - - - - 
echo "$installer Installing udev rules"
sudo cp -f install/10-l_port.rules /etc/udev/rules.d/10-l_port.rules
sudo cp -f install/10-r_port.rules /etc/udev/rules.d/10-r_port.rules
sudo cp -f install/10-dxl_test.rules /etc/udev/rules.d/10-dxl_test.rules
sudo udevadm control --reload

#  - - - - - - - - - Port Permissions  - - - - - - - - - 
# Dialout group
echo "$installer Add user to dialout group"
sudo usermod -a -G dialout "$USER"
# Port permissions
echo "$installer Add ports permissions"

# l_port
if [ -e /dev/bender/l_port ];
then
   real_port=$(readlink -f '/dev/bender/l_port')
   echo "$installer Port /dev/bender/l_port connected at ${real_port}"
   echo "$installer Add ports permissions"
   sudo chmod a+rw ${real_port}
else
   echo "$installer ${red}Port /dev/bender/l_port NOT connected.${reset}"
   echo "$installer ${yellow}You must add permissions using \$ sudo chmod a+rw \$(readlink -f '/dev/bender/l_port')${reset}"
fi
# r_port
if [ -e /dev/bender/r_port ];
then
   real_port=$(readlink -f '/dev/bender/r_port')
   echo "$installer Port /dev/bender/r_port connected at ${real_port}"
   echo "$installer Add ports permissions"
   sudo chmod a+rw ${real_port}
else
   echo "$installer ${red}Port /dev/bender/r_port NOT connected.${reset}"
   echo "$installer ${yellow}You must add permissions using \$ sudo chmod a+rw \$(readlink -f '/dev/bender/r_port')${reset}"
fi

# dxl_test
if [ -e /dev/bender/dxl_test ];
then
   real_port=$(readlink -f '/dev/bender/dxl_test')
   echo "$installer Port /dev/bender/dxl_test connected at ${real_port}"
   echo "$installer Add ports permissions"
   sudo chmod a+rw ${real_port}
else
   echo "$installer ${red}Port /dev/bender/dxl_test NOT connected.${reset}"
   echo "$installer ${yellow}You must add permissions using \$ sudo chmod a+rw \$(readlink -f '/dev/bender/dxl_test')${reset}"
fi

echo "$installer ${yellow}The computer must be restarted in order to complete the installation${reset}"
