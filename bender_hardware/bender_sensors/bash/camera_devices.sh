#!/bin/sh
#
# Setea variables de ambiente con los camera ids
# 
# matias.pavez.b@gmail.com


# - - -  available data - - - 

# - device id's -
BENDER_CAMERA_DEVICE_ID_ANY='0'

# logitech id's
BENDER_CAMERA_IDS[0]='BA1BA220'
BENDER_CAMERA_IDS[1]='444EC120'
BENDER_CAMERA_IDS[2]='1F95B220' # matias's camera


# ______________________________________________________________________
# - - - User Configuration - - -

# - select device id's -
# head camera id's
BENDER_CAMERA_HEAD_ID=$BENDER_CAMERA_DEVICE_ID_ANY

# ______________________________________________________________________

# - - - Current Bender Devices - - -  

# - export valid device id's -
# head id's
export BENDER_CAMERA_HEAD_ID


# ______________________________________________________________________

# - - - Clean Up - - -  

unset BENDER_CAMERA_DEVICE_ID_ANY
unset BENDER_CAMERA_IDS
