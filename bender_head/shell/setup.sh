#!/bin/sh

# useful variables
_THIS_DIR="$(rospack find bender_head)/shell"

alias bender_hwcheck_head="rosrun bender_head hw_check.py"

# Source shell tools OBSOLETE!
#. "$_THIS_DIR"/shell_tools.sh

unset _THIS_DIR