#!/bin/bash
#
# run me like this:
# > cdb bender_tts
# > bash install/install.sh
#

# - - - - - - S E T U P - - - - - - - -
# # # # # # # # # # # # # # # # # # # #

# Useful Variables
pkg_name="bender_tts"
pkg_path=$(rospack find "$pkg_name")
install_space="$BENDER_WS"/install/soft/hri/speech
mkdir -p "$install_space" && cd "$install_space"

# - - - - - - I N S T A L L - - - - - -
# # # # # # # # # # # # # # # # # # # #

## ensure we have all required files
# locations
tarfile="$install_space"/speech.tar.gz
tarfolder="$install_space"/files
megadownfiles="$install_space"/.megadown

tarfile_short="\"\$BENDER_WS\"/${tarfile#$BENDER_WS/}"
tarfolder_short="\"\$BENDER_WS\"/${tarfolder#$BENDER_WS/}/"
if [ ! -d "$tarfolder" ] || [ ! $(ls -A "$tarfolder" | wc -l) ]; then
	rm -rf "$tarfolder"
	rm -rf "$megadownfiles"
	echo " - speech install files NOT found on path: $tarfolder_short"

	if [ ! -e "$tarfile" ]; then
		echo " - tar file NOT found: $tarfile_short"

		# retrieve install files from mega
		echo " - ... retrieving tarfile from mega"
		"$BENDER_SYSTEM"/bash/megadown/megadown 'https://mega.nz/#!vltxCDoB!ZncFt39E9QMfCNW8-we7O7veBjlmKaAezcqrhbdYUDM'
		rm -rf "$megadownfiles"

	else
		echo " - tar file found: $tarfile_short"

		if ! tar -tf "$tarfile" &> /dev/null; then
			echo " - it seems the tar file is corrupted. retrieving again..."
			rm -rf "$tarfile"
			"$BENDER_SYSTEM"/bash/megadown/megadown 'https://mega.nz/#!vltxCDoB!ZncFt39E9QMfCNW8-we7O7veBjlmKaAezcqrhbdYUDM'
			rm -rf "$megadownfiles"
		fi
	fi
	if ! tar -tf "$tarfile" &> /dev/null; then
		echo " - the tar file is corrupted. solve this!"
		echo " - ... deleting tar: $tarfile ..."
		echo " - ... deleting megadown data: $tarfile ..."
		rm -rf "$tarfile"
		rm -rf "$megadownfiles"
		echo " - BYE! ..."
		exit 1
	fi
	echo " - ... extracting speech files"
	mkdir -p "$tarfolder"
	tar -xf "$tarfile" -C "$install_space"
	echo " - ... OK"
else
	echo " - speech install files found on path: $tarfolder_short"
fi


# install synthesizer
bash "$pkg_path"/install/install_synthesizer.sh


echo ""
echo "Done. ;)"
echo ""
# :)
