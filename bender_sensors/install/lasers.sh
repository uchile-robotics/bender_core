#!/bin/bash

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #
source "$UCHILE_WS"/system/install/util/pkg_install.bash
installer="[INSTALLER]:"

# urg_node fork install
# TODO: unificar como metodo llamado desde bender_system 
# (esto se usa en el instalador de bender)
uchile_cd forks
if [ ! -d urg_node ]; then
  echo "Cloning -urg_node- fork from github."
  git clone https://github.com/uchile-robotics/urg_node.git
  cd urg_node
  git checkout master
else
  echo "-urg_node- fork already exists. updating"
  cd urg_node
  git checkout -- .
  git fetch
  git checkout uchile-devel
fi

#  - - - - - - - - - Install Rules - - - - - - - - - - 
echo -e "\n$installer Installing udev rules for the hokuyo laser range finders";

# prepare folder for udev rules
uchile_cd forks
cd ..
catkin_make

uchile_cd forks
sudo mkdir -p /opt/bender/udev
sudo cp ../devel/lib/urg_node/getID /opt/bender/udev/

uchile_cd bender_sensors
sudo cp -f install/files/10-bender_hokuyo.rules /etc/udev/rules.d/10-bender_hokuyo.rules;
sudo udevadm control --reload

