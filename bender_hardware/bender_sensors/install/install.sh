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
source "$BENDER_SYSTEM"/setup.bash

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #
bender_cd bender_sensors
bash install/cameras.sh
bash install/lasers.sh

# falta automatizar descarga de instaladores!
#bash install/rgbd.sh

# :)
