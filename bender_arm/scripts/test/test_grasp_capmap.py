#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from bender_arm_control.srv import AttachObject, AttachObjectRequest
from bender_arm_planning.srv import ManageOctomap, ManageOctomapRequest
from bender_arm_planning.msg import OctomapOptions

from geometry_msgs.msg import Pose, PoseStamped, Point
from uchile_msgs.msg import CylindricalObject
from tf import transformations
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive

def get_pose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

def get_collision_box(pose, name, dim = [0.1,0.1,0.1]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = 'bender/base_link'
    obj.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = dim

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD
    return obj

def get_collision_cylinder(pose, name, dim = [0.22, 0.04]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = 'bender/base_link'
    obj.id = name

    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = dim

    obj.primitives.append(cylinder)
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD
    return obj

def main():
    rospy.wait_for_service('/l_arm_planning/attach_object')
    rospy.ServiceProxy('/l_arm_planning/attach_object', AttachObject)
    #attach_object = rospy.ServiceProxy('/l_arm_planning/attach_object', AttachObject)

    rospy.wait_for_service('/manage_octomap')
    rospy.ServiceProxy('/manage_octomap', ManageOctomap)
    #octomap = rospy.ServiceProxy('/manage_octomap', ManageOctomap)

    # Grasp ob
    pringles_pose = get_pose(0.60, 0.25, 0.75)
    pringles = get_collision_cylinder(pringles_pose, 'pringles', [0.10, 0.04])

    # Actualizar y congelar octomap
    # octomap_opt = OctomapOptions(OctomapOptions.UPDATE)
    # octomap(octomap_opt)

    limb = Limb('l')

    possible_grasp = limb.arm.get_grasp_capmap(pringles)

    if not possible_grasp:
        rospy.logerr('No se encontraron grasps')
        return

    limb.arm.set_position_named('home')
    rospy.sleep(2.0)
    limb.arm.set_position_named('premanip_1')
    rospy.sleep(3.0)

    rospy.loginfo('Se encontraron {} grasps'.format(len(possible_grasp['pregrasp'])))
    # Escoger grasps
    pregrasp_joints = possible_grasp['pregrasp'][0]
    grasp_joints = possible_grasp['grasp'][0]

    result = limb.arm.set_joint(pregrasp_joints) # Movimiento con planificador

    if (result.error_code.val == MoveItErrorCodes.SUCCESS):
        limb.gripper.open(effort = 300)
        rospy.sleep(1.0)

        limb.arm.move_joint(grasp_joints, interval = 2.5) # Movimiento con collisiones permitidas
        limb.arm.wait()
        rospy.sleep(0.5)

        limb.gripper.close(effort = 300)
        rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()
    