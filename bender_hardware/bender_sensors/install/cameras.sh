#!/bin/bash

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

UDEV_DIR="/opt/bender/udev"
RULES_DIR="/etc/udev/rules.d"

bender_cd bender_sensors

# -- udev rules --
echo -e "\n[INSTALLER]: Installing udev rules for Logitech cameras"

# prepare script for udev rules
sudo mkdir -p "$UDEV_DIR"
sudo cp -f install/files/camera.sh "$UDEV_DIR"/camera.sh

# add udev rules
sudo cp -f install/files/bender_camera.rules "$RULES_DIR"/bender_camera.rules
sudo udevadm control --reload