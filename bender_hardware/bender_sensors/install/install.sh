#!/bin/bash

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

bender_cd bender_sensors
bash install/cameras.sh
bash install/lasers.sh

# falta automatizar descarga de instaladores!
#bash install/rgbd.sh

# :)
