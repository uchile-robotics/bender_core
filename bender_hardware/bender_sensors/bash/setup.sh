#!/bin/sh


# useful variables
_THIS_DIR="$(rospack find bender_sensors)/bash"

# source camera devices
. "$_THIS_DIR"/camera_devices.sh

# source depth devices
. "$_THIS_DIR"/depth_devices.sh

unset _THIS_DIR