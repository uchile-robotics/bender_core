#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #
UDEV_DIR="/opt/bender/udev"
RULES_DIR="/etc/udev/rules.d"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# usb_cam fork install
# TODO: unificar como metodo llamado desde bender_system 
# (esto se usa en el instalador de bender)
bender_cd forks
if [ ! -d usb_cam ]; then
  echo "Cloning -usb_cam- fork from github."
  git clone https://github.com/uchile-robotics/usb_cam.git
  cd usb_cam
  git checkout master
else
  echo "-usb_cam- fork already exists. updating"
  cd usb_cam
  git checkout -- .
  git fetch
  git checkout 0.3.4
fi


bender_cd bender_sensors

# -- udev rules --
echo -e "\n$installer Installing udev rules for Logitech cameras"

# prepare script for udev rules
sudo mkdir -p "$UDEV_DIR"
sudo cp -f install/files/camera.sh "$UDEV_DIR"/camera.sh

# add udev rules
sudo cp -f install/files/10-bender_camera.rules "$RULES_DIR"/10-bender_camera.rules
sudo udevadm control --reload