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
installer="[INSTALLER]:"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #
uchile_cd bender_sensors
source install/cameras.sh
source install/lasers.sh

# falta automatizar descarga de instaladores!
#source install/rgbd.sh

# :)
