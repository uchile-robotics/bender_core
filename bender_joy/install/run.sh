#!/bin/sh
#
# mpavez

# para poder mostrar notificaciones
export XAUTHORITY=/home/mpavez/.Xauthority
export DISPLAY=:0.0

# lock file: indica que el joystick esta conectado
DEVFILE=/opt/bender/udev/joy.lock

if [ "$ACTION" = "add" ]; then

	if [ ! -e "$DEVFILE" ]; then
		# echo "$ACTION {add}" >> "/home/mpavez/log.txt"
		mkdir /opt/bender
		mkdir /opt/bender/udev
		touch "$DEVFILE"
		notify-send Hardware "Joystick Adapter Connected" -t 1000 -i /home/mpavez/face_icon.png
	fi
	
elif [ "$ACTION" = "remove" ]; then
	
	if [ -e "$DEVFILE" ]; then
		# echo "$ACTION {remove}" >> "/home/mpavez/log.txt"
		rm -f "$DEVFILE"
		notify-send Hardware "Joystick Adapter Disconnected" -t 1000 -i /home/mpavez/face_icon.png	
	fi

fi
