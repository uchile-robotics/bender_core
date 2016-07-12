#!/usr/bin/env sh

# Line tittle
installer="[bender_fieldbus]:"

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
#sudo chmod a+rw /dev/bender/l_port
#sudo chmod a+rw /dev/bender/r_port
echo "$installer The computer must be restarted in order to complete the installation"
