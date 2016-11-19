#!/bin/bash

# bender_description package installer
# run: $ bash bender_description/install/install.sh

#  - - - - - - - - - Setup - - - - - - - - - - - 
# Color
red=$(tput setaf 1)
green=$(tput setaf 2)
yellow=$(tput setaf 3)
reset=$(tput sgr0)
bold=$(tput bold)

# Line tittle
installer="${bold}[bender_description]:${reset}"

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
# Download bender_description meshes
echo "$installer Downloading mesh files"
"$BENDER_SYSTEM"/bash/megadown/megadown 'https://mega.nz/#!PgMFnagA!yf6tneLj5p0IA5SLbR400eZaBeVUo2pLMNZWTSXYxls'
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error downloading mesh files.${reset}"
    exit 1 # Terminate and indicate error
else
    echo "$installer Mesh files downloaded successfully."
fi

#  - - - - - - - - - Extract files - - - - - - - - - - -
# Extract files
echo "$installer Extracting mesh files"
unzip -q -o bender_description.zip -d "$BENDER_WS"/base_ws/src/bender_common/
if [ $? -ne 0 ]; then
    echo "$installer ${red}Error extracting mesh files.${reset}"
    exit 1 # Terminate and indicate error
else
    echo "$installer Mesh files extracted successfully."
fi

