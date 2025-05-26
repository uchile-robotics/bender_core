#!/bin/bash
RESET=$(tput sgr0)
HIGHLIGHT=$(tput setaf 3)
HIGHLIGHT2=$(tput setaf 6)
default_ns="Matilde"
echo "Namespace: ( [Enter] to use the default value ""$HIGHLIGHT2""$default_ns""$RESET"" )"
read ns
if [[ -z "$ns" ]]; then
	ns=$default_ns
fi
echo "Assuming namespace: ""$HIGHLIGHT2""$ns"
echo "$HIGHLIGHT""ros2 run rosaria2 emergency_stop --ros-args -p namespace:=$ns"
echo "$RESET"
ros2 run rosaria2 emergency_stop --ros-args -p namespace:=$ns
