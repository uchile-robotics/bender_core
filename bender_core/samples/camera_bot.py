#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bender_core.robot import Robot
from bender_core.core.camera import CameraSkill


def build_robot():
    robot = Robot("camera_bot")
    # Add head
    robot.set(CameraSkill.get_instance())
    if not robot.check():
        raise RuntimeError("Required skills dont available")
    # Run setup
    if not robot.setup():
        raise RuntimeError("Robot setup failed")
    return robot

if __name__ == "__main__":
    rospy.init_node("camera_bot_example")
    robot = build_robot()

    skill = robot.get("camera")
    msgimage = skill.get_last_image()
    cv_image = skill.imgmsg_to_cv2(msgimage)
    cv_image_cartoon = skill.apply_effect_cartoon(cv_image)
    skill.display(cv_image_cartoon)

    # skill.save_last_img("selfie.jpg")
    skill.cvimg_to_file(cv_image_cartoon, "selfie.jpg")

    robot.shutdown()
