#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
pkg_name="bender_sensors"
install_path=$(rospack find $pkg_name)/install
install_files="$install_path"/files

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

UDEV_DIR="/opt/bender/udev"
RULES_DIR="/etc/udev/rules.d"

# -- udev rules --
echo -e "\n[INSTALLER]: Installing udev rules for Logitech cameras"

# prepare script for udev rules
sudo mkdir -p "$UDEV_DIR"
sudo cp -f "$install_files"/camera.sh "$UDEV_DIR"/camera.sh

# add udev rules
sudo cp -f "$install_files"/bender_camera.rules "$RULES_DIR"/bender_camera.rules
sudo udevadm control --reload