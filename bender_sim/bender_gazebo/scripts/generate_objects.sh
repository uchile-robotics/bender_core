#!/bin/sh

# TODO: poner como se usa!

rosrun xacro xacro.py "$1" > model.urdf && cat model.urdf && rosrun urdfdom_py display_urdf model.urdf
