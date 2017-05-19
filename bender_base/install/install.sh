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
# # # # # # # # # # # # # # # # # # # #
source "$UCHILE_WS"/system/install/util/pkg_install.bash

# Useful Variables
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

#  - - - - - - - - - Install Rules - - - - - - - - - - 

echo -e "\n$installer Installing udev rules for the pionner\n"
uchile_cd bender_base
sudo cp -f install/files/bender_pioneer.rules /etc/udev/rules.d/bender_pioneer.rules
sudo udevadm control --reload


#  - - - - - - - - - Port Permissions - - - - - - - - - - 

echo -e "\n$installer Port permissions\n"
var_group=$(groups "$USER" | grep -o -w -c dialout)
if [ "$var_group" = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\n$installer add to dialout\n"
  sudo usermod -a -G dialout "$USER"

  echo "$installerThe computer must be restarted in order to complete the installation"
  echo -e "$installer... Do you want to reboot the pc now?\n"
  echo "$installer(y)es (n)o:"
  read var_reboot

  if [ "$var_reboot" = "y" ]; then

    echo -e "\nDone.\n ;)\n"
    sudo shutdown -r now

  else
    echo "$installer... Remember to reboot the computer!!!..."
    echo -e "\n$installer Done.\n :)\n"
  fi
else
  echo -e "\n$installer ... already in dialout group (OK)\n"
fi

# :)
