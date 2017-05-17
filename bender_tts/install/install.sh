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


## ----------------------------------------------------------------------------
## SETUP
## ----------------------------------------------------------------------------
source "$BENDER_WS"/bender_system/install/pkg_install.bash
THIS_SCRIPT=$(readlink -f "$0")
THIS_FOLDER=$(dirname "$THIS_SCRIPT")
source "$THIS_FOLDER"/settings.bash


## ----------------------------------------------------------------------------
## INSTALL
## ----------------------------------------------------------------------------
mkdir -p "$install_space" && cd "$install_space"

if [ -e "$install_token" ]; then
	echo " - mbrola_voices is already installed"
	exit 0
fi

## Download
rm -rf "$tarfolder"
rm -rf "$megadown_tmp"
if [ ! -e "$tarfile" ]; then
	echo " - tar file NOT found: $tarfile"

	# retrieve install files from mega
	echo " - ... retrieving tarfile from mega"
	"$megadown_exe" "$megadown_files"
	rm -rf "$megadown_tmp"

else
	echo " - tar file found: $tarfile"

	if ! tar -tf "$tarfile" &> /dev/null; then
		echo " - it seems the tar file is corrupted. retrieving again..."
		rm -rf "$tarfile"
		"$megadown_exe" "$megadown_files"
		rm -rf "$megadown_tmp"
	fi
fi

## extract
if ! tar -tf "$tarfile" &> /dev/null; then
	echo " - the tar file is corrupted. solve this!"
	echo " - ... deleting tar: $tarfile ..."
	echo " - ... deleting megadown data: $tarfile ..."
	rm -rf "$tarfile"
	rm -rf "$megadown_tmp"
	echo " - BYE! ..."
	exit 1
fi
echo " - ... extracting speech files"
tar -xf "$tarfile" -C "$install_space"
echo " - ... OK"


## install synthesizer
bender_cd bender_tts
source "$pkg_path"/install/install_synthesizer.bash
rm -rf "$tarfolder" 

echo ""
echo "done"
echo ""
