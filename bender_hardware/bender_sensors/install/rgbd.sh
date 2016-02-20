#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

source "$BENDER_CONFIG"/setup.bash ;

# Useful Variables
pkg_name="bender_sensors";
install_path=$(rospack find $pkg_name)/install;
install_files="$install_path"/files;
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - - - ROS packages  - - - - - - - - - - -
# RGBD drivers
sudo apt-get install ros-indigo-openni-launch ;
sudo apt-get install ros-indigo-openni2-launch ;

# - - - - - - - - - NI Patch for Kinect - - - - - - - - - - -
echo -e "\n$installer Installing NI Patch for Kinect device . . ."

# 0: Required file
nipatch="Sensor-Bin-Linux-x64-v5.1.2.1"
nipatch_compressed="SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2"

# 1: Get file
cd "$install_files"
if [ ! -f "$nipatch_compressed" ]; then
  # Control will enter here if the file doesn't exists.

  echo "$installer [WARNING] . . . Download file $nipatch_compressed by hand and try again"
  echo "$installer [WARNING] . . . the file should be inside the folder $install_files"
  # Download "nipatch_compressed" from somewhere
  #wget path_to_file_online ...
  # TODO

fi

# 2: apply patch
if [ -f "$nipatch_compressed" ]; then
  
  echo -e "$installer Installing patch . . . \n"

  # Uncompress:
  tar -xf "$nipatch_compressed"
  
  # install
  cd "$nipatch"
  sudo ./install.sh

  # clean
  cd "$install_files"
  rm -rf "$nipatch"

else
  echo "$installer [WARNING] . . . $install_files/$nipatch_compressed not found!, make sure it gets downloaded and try again"
fi


# - - - - - - - - - NITE - - - - - - - - - - -
# Descarga de NITE:
# http://www.mira-project.org/downloads/3rdparty/bin-linux/
# http://answers.ros.org/question/42654/openninite-incompatible-in-fuertegroovy-on-precise/

echo -e "\n$installer Installing NITE . . ."


# 0: Required file
nite="NITE-Bin-Dev-Linux-x64-v1.5.2.21"
nite_compressed="nite-bin-linux-x64-v1.5.2.21.tar.bz2"

# 1: Get file
cd "$install_files"
if [ ! -f "$nite_compressed" ]; then
  # Control will enter here if the file doesn't exists.

  echo "$installer [WARNING] . . . Download file $nite_compressed by hand and try again"
  echo "$installer [WARNING] . . . the file should be inside the folder $install_files"
  # Download "nite_compressed" from somewhere
  #wget path_to_file_online ...
  # TODO
fi

# 2: apply patch
if [ -f "$nite_compressed" ]; then
  
  echo -e "$installer Installing patch . . . \n"

  # Uncompress:
  tar -xf "$nite_compressed"
  
  # install
  cd "$nite"
  sudo ./uninstall.sh
  sudo ./install.sh

  # clean
  cd "$install_files"
  rm -rf "$nite"

else
  echo "$installer [WARNING] . . . $install_files/$nite_compressed not found!, make sure it gets downloaded and try again"
fi

# THIS IN NOT USED ANYMORE! (but could be useful later)
# forbid loading of kinect modules
#sudo modprobe -r gspca_kinect
#sudo modprobe -r gspca_main
#echo "blacklist gspca_kinect" | sudo tee -a /etc/modprobe.d/blacklist.conf
