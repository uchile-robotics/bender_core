#!/bin/bash

# run: $ ./scripts/install.sh

# The directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# the temp directory used, within $DIR
WORK_DIR=`mktemp -d -p "$DIR"`
# deletes the temp directory
function cleanup {
  rm -rf "$WORK_DIR"
  echo "Deleted temp working directory $WORK_DIR"
}
# Register the cleanup function to be called on the EXIT signal
trap cleanup EXIT
cd $WORK_DIR


# Download bender_description meshes
$BENDER_SYSTEM/bash/megadown/megadown 'https://mega.nz/#!a0930ZYB!sciM89vyDLtrpDpC5_CB-6BCXA6OE1p2Bzq8R6rWeRU'
# Extract files
unzip bender_description.zip -d $BENDER_WS/base_ws/src/bender_common/


