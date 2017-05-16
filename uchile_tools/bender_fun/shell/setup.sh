#!/bin/sh

##############################################################################################
#   shell utilities
##############################################################################################

# deprecated until creation of installation script
return 0


# fun
bender_fun () {

	if hash zsnes 2>/dev/null; then
        echo -e ":)"
    else
    	echo -e "Installing zsnes emulator :)"
        sudo apt-get install zsnes
    fi

    if hash zsnes 2>/dev/null; then
    	bender_cd bender_fun
    	zsnes database/games/fun.zip
    fi
}
