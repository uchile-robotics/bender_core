#!/bin/bash

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
pkg_name="bender_sensors"
install_path=$(rospack find $pkg_name)/install

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

# - - - - - - - - - Install cameras  - - - - - - - - - - -
. "$install_path"/cameras.sh
. "$install_path"/lasers.sh
. "$install_path"/rgbd.sh

# :)
