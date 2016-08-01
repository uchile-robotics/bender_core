#!/usr/bin/bash

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
sudo cp -f install/l_port.rules /etc/udev/rules.d/l_port.rules
sudo cp -f install/r_port.rules /etc/udev/rules.d/r_port.rules
sudo udevadm control --reload

#  - - - - - - - - - Port Permissions  - - - - - - - - - 
# Dialout group
echo "$installer Add user to dialout group"
sudo usermod -a -G dialout "$USER"
# Port permissions
echo "$installer Add ports permissions"
if [ -f /dev/bender/l_port ];
then
   echo "Port /dev/bender/l_port connected."
   echo "Add ports permissions."
   sudo chmod a+rw /dev/bender/l_port
else
   echo "$installer ${red}Port /dev/bender/l_port NOT connected.${reset}"
   echo "$installer ${yellow}You must add permissions using $ sudo chmod a+rw /dev/bender/l_port${reset}"
fi

if [ -f /dev/bender/r_port ];
then
   echo "$installer Port /dev/bender/r_port connected."
   echo "$installer Add ports permissions."
   sudo chmod a+rw /dev/bender/r_port
else
   echo "$installer ${red}Port /dev/bender/r_port NOT connected.${reset}"
   echo "$installer ${yellow}You must add permissions using $ sudo chmod a+rw /dev/bender/r_port${reset}"
fi

echo "$installer ${yellow}The computer must be restarted in order to complete the installation${reset}"
