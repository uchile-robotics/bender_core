#!/bin/sh
#
# Setea variables de ambiente con los kinects_id
# 
# matias.pavez.b@gmail.com


# - - -  available data - - - 

# - device types -
BENDER_DEVICE_ASUS='asus'
BENDER_DEVICE_KINECT='kinect'

# - device id's -
BENDER_RGBD_DEVICE_ID_ANY='#1'

# asus id's
BENDER_ASUS_IDS[0]='1404280205'
BENDER_ASUS_IDS[1]='1407150547'

# kinect id's
BENDER_KINECT_IDS[0]='A00366906471051A' # 
BENDER_KINECT_IDS[1]='A00365808192051A' # 
BENDER_KINECT_IDS[2]='B00364626688048B' # 
BENDER_KINECT_IDS[3]='A00365906656051A' # 
BENDER_KINECT_IDS[4]='A00363809752042A' # 
BENDER_KINECT_IDS[5]='A00365909120051A' #
BENDER_KINECT_IDS[6]='A00366802376051A' # Matias

# ______________________________________________________________________

# - - - User Configuration - - -

# - select device type -
BENDER_RGBD_DEVICE_HEAD=$BENDER_DEVICE_ASUS
BENDER_RGBD_DEVICE_WAIST=$BENDER_DEVICE_ASUS

# - select device id's -
# asus id's
BENDER_ASUS_HEAD_ID=$BENDER_RGBD_DEVICE_ID_ANY
BENDER_ASUS_WAIST_ID=$BENDER_RGBD_DEVICE_ID_ANY

# kinect id's
BENDER_KINECT_HEAD_ID=$BBENDER_RGBD_DEVICE_ID_ANY
BENDER_KINECT_WAIST_ID=$BENDER_RGBD_DEVICE_ID_ANY

# ______________________________________________________________________

# - - - Current Bender Devices - - - 

# - export device type -
export BENDER_RGBD_DEVICE_HEAD
export BENDER_RGBD_DEVICE_WAIST 

# - export valid device id's -
# head id's
if [ "$BENDER_RGBD_DEVICE_HEAD" = "$BENDER_DEVICE_ASUS" ]; then
	export BENDER_RGBD_HEAD_ID="$BENDER_ASUS_HEAD_ID"
elif [ "$BENDER_RGBD_DEVICE_HEAD" = "$BENDER_DEVICE_KINECT" ]; then
	export BENDER_RGBD_HEAD_ID="$BENDER_KINECT_HEAD_ID"
else
	export BENDER_RGBD_HEAD_ID=''
fi

# waist id's
if [ "$BENDER_RGBD_DEVICE_WAIST" = "$BENDER_DEVICE_ASUS" ]; then
	export BENDER_RGBD_WAIST_ID="$BENDER_ASUS_WAIST_ID"
elif [ "$BENDER_RGBD_DEVICE_WAIST" = "$BENDER_DEVICE_KINECT" ]; then
	export BENDER_RGBD_WAIST_ID="$BENDER_KINECT_WAIST_ID"
else
	export BENDER_RGBD_WAIST_ID=''
fi


# _____________________________________________________________________

# - - - Clean Up - - -

unset BENDER_DEVICE_ASUS
unset BENDER_DEVICE_KINECT
unset BENDER_RGBD_DEVICE_ID_ANY
unset BENDER_ASUS_IDS
unset BENDER_KINECT_IDS
unset BENDER_KINECT_HEAD_ID
unset BENDER_ASUS_HEAD_ID
unset BENDER_KINECT_WAIST_ID
unset BENDER_ASUS_WAIST_ID


