# Bender core

## Overview

This repository contain all ROS packages that make up the core of Bender. These correspond to low-level functionalities that are expected to be always available for the users.

This repo follows a certain structure that must be complied to keep these packages usable.

## Structure

- bender_arm: 
- bender_base: launch files and parameters for Bender's mobile base, the pioneer 3AT.
- bender_calibrate: Files for calibrating bender's cameras.
- bender_description: Publishes Bender's TF when running on the real robot. Also contains the corresponding meshes
- bender_gripper
- bender_hand
- bender_head
- bender_joy
- bender_sensors: Contains all the drivers of the sensors used by Bender, like normal and depth cameras and lidar.
- bender_sim
- bender_sound
- bender_tts

Every package has the following structure: