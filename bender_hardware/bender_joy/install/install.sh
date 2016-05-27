#!/bin/bash


# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Permisos para lectura/escritura Joysticks
# ---------------------------------------------------------------
echo -e "\nInstalling Joystick (must be connected)"
if [ ! -c "/dev/input/js0" ]; then
  # Joystick not connected
  echo -e "Joystick isn't connected!!"
  echo -e ". . ."
  echo -e " 1.- Connect the Joy"
  echo -e " 2.- Type in terminal:\n   $ sudo chmod a+rw /dev/input/js0"
  echo -e "Thats it"
  echo -e "\n\n\n"
else 
  # Joystick not connected!
  sudo chmod a+rw /dev/input/js0
fi

# Permisos para lectura de puertos
# ---------------------------------------------------------------
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

# :)
