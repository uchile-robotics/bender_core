#!/bin/bash
#
# prefer running: 
# > cdb bender_speech
# > bash install/install.sh
#


# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
install_space="$BENDER_WS"/install/soft/hri/speech
mkdir -p "$install_space" && cd "$install_space"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #


echo ""
echo "Installing speech synthesizer"
echo "-----------------------------"
echo ""

## Download and install dependences

# speech synthesizer (festival):
sudo apt-get install festival festlex-cmu festlex-poslex festlex-oald libestools1.2

# other tools
sudo apt-get install unzip


# Installing voices for the speech synthesis system
# ==================================================
# installing guide: http://ubuntuforums.org/showthread.php?t=751169
#
# bender currently uses two voices:
# - english: us2_mbrola (voice_us2_mbrola)
# - spanish: el_diphone (voice_el_diphone)


## A.- Diphone voices 
## - - - - - - - - - - - - - - -
# this voices:
# - poor quality
# - there are two versions of them: 8kHz and 16kHz (prefer this!)
#
# demos: http://festvox.org/voicedemos.html

# american-english 16kHz
#sudo apt-get install festvox-don festvox-rablpc16k festvox-kallpc16k festvox-kdlpc16k

# español-castellano (el_diphone)
sudo apt-get install festvox-ellpc11k


## B.- MBROLA voices
## - - - - - - - - - - - - - - -
#
# - good quality voices
# - download link and demos: http://tcts.fpms.ac.be/synthesis/mbrola/mbrcopybin.html
# - to use the MBROLA voices we need three parts:
#     * the mbrola parser
#     * the mbrola voice
#     * a voice specific wrapper for festival
#
# - mbrola voices needs "libc6" package. 
#      If you already have a 64bit version of it, the package manager could
#      ask you to install the i386 version. Try using this command:
#           $ sudo apt-get -f install
#
# Download files from:
# mbrola        : http://tcts.fpms.ac.be/synthesis/mbrola/bin/pclinux/mbrola3.0.1h_i386.deb
# english voice : http://tcts.fpms.ac.be/synthesis/mbrola/dba/us2/us2-980812.zip
# voice wrapper : http://www.festvox.org/packed/festival/latest/festvox_us2.tar.gz
festvox_path="/usr/share/festival/voices"
festvox_path_en="$festvox_path"/english


# Install the mbrola parser
mbrola_folder="$install_space"/files/mbrola_voices
if [ -d "$mbrola_folder" ]; then

	install_token="$install_space"/INSTALLED_MBROLA_VOICES
	if [ ! -e "$install_token" ]; then
	
		cd "$mbrola_folder"
		sudo dpkg -i mbrola3.0.1h_i386.deb

		# Unzip/tar
		rm -rf us2; unzip -x us2-980812.zip
		tar xvf festvox_us2.tar.gz

		# Install voice and wrapper
		sudo rm -rf $festvox_path_en/us2_mbrola/*; sudo mkdir -p $festvox_path_en/us2_mbrola/
		sudo mv us2 $festvox_path_en/us2_mbrola/
		sudo mv -f festival/lib/voices/english/us2_mbrola/* $festvox_path_en/us2_mbrola/

		# clean
		rm -rf us2/
		rm -rf festival/

		# mark as installed
		touch "$install_token"
	else
		echo " - mbrola_voices is already installed"
	fi
else
	echo " - mbrola folder not found!: $mbrola_folder"
	echo "   please run install.sh"
	exit 1
fi


## C.- CMU Arctic voices
## - - - - - - - - - - - - - - - -
#
# - better sound quality than mbrola and diphone voices
# - each voice takes a lot of space in disk (~100MB)
# - demos: http://festvox.org/voicedemos.html

## D.- Enhanced Nitech HTS voices
## - - - - - - - - - - - - - - - - 
#
# - better sound quality than CMU arctic voices
# - each voice takes very little disk space :)
# - demos: http://festvox.org/voicedemos.html


echo ""
echo "Speech Synthesizer install: Done."
echo ""

# :)
