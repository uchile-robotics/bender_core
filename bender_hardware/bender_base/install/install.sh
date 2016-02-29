#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
install_path=$(rospack find $pkg_base)/install
install_files=$install_path/files
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Rosaria install --> chequeo del fork y rama / tag  ok

#  - - - - - - - - - Install Rules - - - - - - - - - - 

echo -e "\n$installer Installing udev rules for the pionner\n"
sudo cp -f "$install_files"/bender_pioneer.rules /etc/udev/rules.d/bender_pioneer.rules
sudo udevadm control --reload


#  - - - - - - - - - Port Permissions - - - - - - - - - - 

echo -e "\n$installer Port permissions\n"
var_group=$(groups "$USER" | grep -o -w -c dialout)
if [ "$var_group" = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\n$installerPort permissions\n"
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
fi

# :)
