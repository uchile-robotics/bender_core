#!/bin/bash
# Create URDF and SDF files from xacro

# run: ./scripts/update_models.sh

BASEDIR=$(dirname "$0")
cd "$BASEDIR"/../robots
rm -rf bender.urdf
rm -rf bender.sdf
echo "Exporting URDF..."
rosrun xacro xacro.py bender_asus.urdf.xacro prefix:="bender/" > bender.urdf
echo "Exporting SDF..."
gzsdf print bender.urdf > bender.sdf