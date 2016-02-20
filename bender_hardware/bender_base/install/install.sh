#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

source "$BENDER_CONFIG"/setup.bash ;

# Useful Variables
pkg_name="bender_base";
install_path=$(rospack find $pkg_name)/install;
install_files=$install_path/files;
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - - - Rosaria - - - - - - - - - - -

echo -e "\n$installer Installing RosAria\n"

# install
cd "$BENDER_WORKSPACE"
if [ -d "rosaria" ]; then
  # Control will enter here if the directory exists.
  echo "$installer Deleting old 'rosaria' git repository . . ."
  rm -rf rosaria
fi
echo "$installer Cloning rosaria repository on 'rosaria' folder . . ."
git clone https://github.com/amor-ros-pkg/rosaria.git rosaria


cd "$BENDER_WORKSPACE"
echo "$installer Updating rosdep and installing rosaria dependencies . . ."
rosdep update
rosdep install rosaria

# Apply patch
# hacerlo manualmente!!..  esperar al fork!
#echo "$installer Applying patch to rosaria sources . . ."
#cp -f $install_files/RosAriaPatch.cpp $BENDER_WORKSPACE/rosaria/RosAria.cpp;

cd "$BENDER_WORKSPACE"/..
catkin_make --only-pkg-with-deps rosaria


#  - - - - - - - - - Install Rules - - - - - - - - - - 

echo -e "\n$installer Installing udev rules for the pionner\n";
sudo cp -f "$install_files"/bender_pioneer.rules /etc/udev/rules.d/bender_pioneer.rules;
sudo udevadm control --reload


#  - - - - - - - - - Port Permissions - - - - - - - - - - 

echo -e "\n$installer Port permissions\n";
var_group=$(groups "$USER" | grep -o -w -c dialout);
if [ "$var_group" = "0" ]; then
  # Si no esta en el grupo -> agregar
  echo -e "\n$installerPort permissions\n";
  sudo usermod -a -G dialout "$USER";

  echo "$installerThe computer must be restarted in order to complete the installation";
  echo -e "$installer... Do you want to reboot the pc now?\n";
  echo "$installer(y)es (n)o:";
  read var_reboot;

  if [ "$var_reboot" = "y" ]; then

    echo -e "\nDone.\n ;)\n";
    sudo shutdown -r now;

  else
    echo "$installer... Remember to reboot the computer!!!...";
    echo -e "\n$installer Done.\n ;)\n";
  fi 
fi

# :)
