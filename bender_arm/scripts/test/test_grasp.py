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
    #attach_object = rospy.ServiceProxy('/l_arm_planning/attach_object', AttachObject)
    rospy.ServiceProxy('/l_arm_planning/attach_object', AttachObject)

    rospy.wait_for_service('/manage_octomap')
    octomap = rospy.ServiceProxy('/manage_octomap', ManageOctomap)

    # Table
    # table_pose = get_pose(0.65, 0.0, 0.70)
    # table = get_collision_box(table_pose, 'table', [0.5, 1.0, 0.08])
    # attach_object(table)

    # Grasp ob
    pringles_pose = get_pose(0.70, 0.25, 0.75)
    pringles = get_collision_cylinder(pringles_pose, 'pringles', [0.10, 0.04])
    #attach_object(pringles)

    # Actualizar y congelar octomap
    octomap_opt = OctomapOptions(OctomapOptions.UPDATE)
    octomap(octomap_opt)

    l_limb = Limb('l')

    l_limb.arm.generate_grasp(pringles, axial_res = 6, angle_res = 10)

    l_limb.arm.set_position_named('home')
    rospy.sleep(2.0)
    l_limb.arm.set_position_named('premanip_1')
    rospy.sleep(3.0)

    possible_grasp = l_limb.arm.get_grasp()

    if len(possible_grasp.ik_solutions) == 0:
        rospy.logerr('NO SE GENERARON GRASPS :(')
        return

    pregrasp_joints = possible_grasp.ik_solutions[2*possible_grasp.order[0]].positions
    grasp_joints = possible_grasp.ik_solutions[2*possible_grasp.order[0]+1].positions
    
    result = l_limb.arm.set_joint(pregrasp_joints) # Movimiento con planificador

    if (result.error_code.val == MoveItErrorCodes.SUCCESS):
        l_limb.gripper.open(effort = 300)
        rospy.sleep(1.0)

        l_limb.arm.move_joint(grasp_joints, interval = 2.5) # Movimiento con collisiones permitidas
        l_limb.arm.wait()
        rospy.sleep(0.5)

        l_limb.gripper.close(effort = 300)
        l_limb.gripper.open(effort = 300)
        l_limb.gripper.close(effort = 300)
        l_limb.gripper.open(effort = 300)
        l_limb.gripper.close(effort = 300)
        rospy.sleep(1.0)
    else:
        rospy.sleep(2.0)
        l_limb.arm.set_position_named('premanip_1')
        rospy.sleep(2.0)

    # if possible_grasp.succeed:
    #   l_limb.arm.pick( 'pringles', possible_grasp.grasps, support_name = 'table', planner = 'BKPIECEkConfigDefault')


    # l_limb.arm.set_position_named('premanip_1')
    # rospy.sleep(3.0)
    # l_limb.arm.set_position_named('home')


    # posible_grasp = l_limb.arm.grasp(obj)
    # rospy.loginfo("Cost: " + str(posible_grasp.costs[0]))


    # #print posible_grasp
    # l_limb.arm.move_to_point(posible_grasp.points[0], posible_grasp.joint_names, interval=2.0)
    # print l_limb.arm.wait()

    # print l_limb.gripper.close(effort = 300)
    # l_limb.gripper.open(effort = 300)
    # rospy.sleep(2.0)
    # l_limb.gripper.close(effort = 300)
    # rospy.sleep(2.0)
    # l_limb.arm.set_position_named('home')
    # rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()
    