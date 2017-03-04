#!/bin/bash

# bender_description package installer
# run: $ bash bender_description/install/install.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[bender_description]:${reset}"

# The directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# the temp directory used, within $DIR
WORK_DIR=$(mktemp -d -p "$DIR")
# deletes the temp directory
function cleanup {
    rm -rf "$WORK_DIR"
    echo "$installer Deleted temp working directory $WORK_DIR"
}
# Register the cleanup function to be called on the EXIT signal
trap cleanup EXIT
cd "$WORK_DIR"


#  - - - - - - - - - Download files - - - - - - - - - - - 
BACKUP_FOLDER="$BENDER_WS"/install/base/common/description
BACKUP_FILE=bender_description.zip
BACKUP_FILE_FULL="$BACKUP_FOLDER/$BACKUP_FILE"
if [ ! -r "$BACKUP_FILE_FULL" ]; then
	
	# Download bender_description meshes
	echo "$installer Backup file not found ($BACKUP_FILE_FULL), downloading mesh files ..."
	"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!m0sAUDTA!ZtRQpf0niHwpAzjq8OrcBStvEqngK_RtvXa9TF2vo54'
	if [ $? -ne 0 ]; then
	    echo "$installer ${red}Error downloading mesh files.${reset}"
	    exit 1 # Terminate and indicate error
	fi
	echo "$installer Mesh files downloaded successfully."

	# create backup
	echo "$installer Creating backup on $BACKUP_FILE_FULL"
	mkdir -p "BACKUP_FOLDER"
	cp "$BACKUP_FILE" "$BACKUP_FILE_FULL"
	
else
	echo "$installer A backup file for $BACKUP_FILE already exists on $BACKUP_FOLDER"
	cp "$BACKUP_FILE_FULL" "$WORK_DIR"
fi


#  - - - - - - - - - Extract files - - - - - - - - - - -
# Extract files
echo "$installer Extracting mesh files"
unzip -q -o "$BACKUP_FILE" -d "$BENDER_WS"/base_ws/src/bender_common/
OUT=$?
if [ $OUT -ne 0 ]; then
    echo "$installer ${red}Error extracting mesh files.${reset}"

    # remove backup in error
    echo "removing backup ... "
    rm -f "$BACKUP_FILE_FULL"

    exit 1 # Terminate and indicate error
else
    echo "$installer Mesh files extracted successfully."
fi
