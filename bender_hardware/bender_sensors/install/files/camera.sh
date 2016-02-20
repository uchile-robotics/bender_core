#!/bin/sh

# -- useful commands --
# to search for valid environment variables and values triggered at (dis)connect time
# > udevadm monitor --env   
#
# lookup for udev variables to match when the device is connected
# > sudo udevadm info  --query=path /dev/video0 --attribute-walk
#
# reload a recently modified udev rule
# > sudo udevadm control --reload

DEVFILE=/opt/bender/udev/cameras.info

if [ "$ACTION" = "add" ]; then

	# remove if already exists
	if grep -q "$ID_SERIAL_SHORT" "$DEVFILE"; then
		#echo "($ID_SERIAL_SHORT found), updating to $DEVNAME" >> $DEVFILE
		sed -i '/'"$ID_SERIAL_SHORT"'/d' "$DEVFILE"
	fi

	# yaml style file
	echo "$ID_SERIAL_SHORT: $DEVNAME" >> "$DEVFILE"
	
elif [ "$ACTION" = "remove" ]; then

	if grep -q "$ID_SERIAL_SHORT" "$DEVFILE"; then
		#echo "($ID_SERIAL_SHORT found), removing from list" >> $DEVFILE
		sed -i '/'"$ID_SERIAL_SHORT"'/d' "$DEVFILE"
	fi

fi