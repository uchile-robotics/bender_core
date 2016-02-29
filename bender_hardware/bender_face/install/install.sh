#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
install_path=$(rospack find bender_face)/install
install_files="$install_path"/files


# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# install face rules
sudo cp "$install_files"/bender_head.rules /etc/udev/rules.d/bender_head.rules

# Permisos para lectura de puertos
var_group=$(groups "$USER" | grep -o -w -c dialout)
if [ "$var_group" = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\nPort permissions\n"
  sudo usermod -a -G dialout "$USER"

  echo "The computer must be restarted in order to complete the installation"
  echo -e "... Do you want to reboot the pc now?\n"
  echo "(y)es (n)o:"
  read var_reboot

  if [ "$var_reboot" = "y" ]; then

    echo -e "\nDone.\n ;)\n"
    sudo shutdown -r now

  else
    echo "... Remember to reboot the computer!!!..."
    echo -e "\nDone.\n :)\n"
  fi 
fi

echo -e "\nDone.\n :)\n"
# :)
