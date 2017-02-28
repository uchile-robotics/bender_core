#!/bin/bash

# bender_gazebo package installer
# run: $ bash bender_gazebo/install/install.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[bender_gazebo]:${reset}"

# The directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# the temp directory used, within $DIR
WORK_DIR=`mktemp -d -p "$DIR"`
# deletes the temp directory
function cleanup {
    rm -rf "$WORK_DIR"
    echo "$installer Deleted temp working directory $WORK_DIR"
}
# Register the cleanup function to be called on the EXIT signal
trap cleanup EXIT
cd $WORK_DIR

#  - - - - - - - - - Download files - - - - - - - - - - - 
# Download bender_gazebo meshes
echo "$installer Downloading models"
"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!W5USDRYK!doksKMcdNrF1VVdCr7qrXwp2dFMrrhX8ic8EuEALWqc'
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error downloading mesh files.${reset}"
    exit 1 # Terminate and indicate error
else
    echo "$installer Mesh files downloaded successfully."
fi

#  - - - - - - - - - Extract files - - - - - - - - - - -
# Extract files
echo "$installer Extracting mesh files"
unzip -q -o gazebo_models.zip -d ~
OUT=$?
if [ $OUT -ne 0 ]; then
    echo "$installer ${red}Error extracting mesh files.${reset}"
    exit 1 # Terminate and indicate error
else
    echo "$installer Mesh files extracted successfully."
fi

