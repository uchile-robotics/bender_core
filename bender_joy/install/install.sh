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
  echo -e " 2.- Type in terminal:\n   $ sudo chmod a+rw /dev/input/js*"
  echo -e "Thats it"
  echo -e "\n\n\n"
else 
  # Joystick is connected!
  echo "- giving a+rw permissions to /dev/input/js{0123}!!"
  sudo chmod a+rw /dev/input/js0
  sudo chmod a+rw /dev/input/js1
  sudo chmod a+rw /dev/input/js2
  sudo chmod a+rw /dev/input/js3
fi

# Permisos para lectura de puertos
# ---------------------------------------------------------------
echo "- checking port permissions!!"
var_group=$(groups "$USER" | grep -o -w -c dialout)
if [ "$var_group" = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\n- Giving dialout permissions to $USER\n"
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
else
  echo  "- nothing to do here"
fi
echo  "- OK"

# :)
