#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import sys
import copy
import numpy as np
# ROS Core
import rospy
import actionlib
# ROS Msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from sensor_msgs.msg import JointState
# Msg for planning
from bender_arm.srv import Position, PositionRequest, PositionResponse
from bender_arm.srv import PositionNamed, PositionNamedRequest, PositionNamedResponse
from bender_arm.srv import PositionServoing, PositionServoingRequest, PositionServoingResponse
from bender_arm.srv import GripperOrientation, GripperOrientationRequest, GripperOrientationResponse
from bender_arm.srv import SetPlannerID, SetPlannerIDRequest, SetPlannerIDResponse
from bender_arm.srv import JointTarget, JointTargetRequest, JointTargetResponse

# Capability Map
from bender_arm_planning.srv import CapabilityMapGrasp, CapabilityMapGraspRequest, CapabilityMapGraspResponse
from bender_arm_planning.srv import CapabilityMapBestDistance, CapabilityMapBestDistanceRequest, CapabilityMapBestDistanceResponse

# ACM
from bender_arm_planning.srv import AddObjectACM, AddObjectACMRequest, AddObjectACMResponse

# Grasp
from bender_arm.msg import GraspGeneratorAction, GraspGeneratorGoal
# Pick
from moveit_msgs.msg import PickupGoal, PickupAction, CollisionObject
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from bender_arm.srv import Grasp, GraspRequest, GraspResponse
from bender_arm.checkpoint_planner import CheckpointPlanner


class Arm(object):
    
    joint_names_base = ['shoulder_pitch_joint', 'shoulder_roll_joint',
      'shoulder_yaw_joint', 'elbow_pitch_joint', 'elbow_yaw_joint',
      'wrist_pitch_joint']

    num_joints = 6

    def __init__(self, arm_name):
        """
        Clase Arm, para el manejo de posiciones a nivel de FollowJointTrajectoryAction, no requiere MoveIt!
        @param arm_name: Nombre del brazo, ['r_arm','l_arm']

        Ej. Crear interfaz brazo derecho
        arm = Arm('r_arm')
        """  
        self.name = arm_name
        self.side = arm_name[0]
        self.blind_planner = CheckpointPlanner()
        # Arm namespace
        ns = '/bender/' + self.name + '_controller'
        # Joint names
        self.joint_names = ['{0}_{1}'.format(self.name[0], joint) for joint in Arm.joint_names_base]
        # Client for JTA
        rospy.loginfo('Init ArmClient with {0}/'.format(ns))
        self.jta_client = actionlib.SimpleActionClient(ns + '/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for Joint trajectory action server for {0}'.format(self.name))
        # Wait 5 Seconds for the JTA server to start or exit
        if not self.jta_client.wait_for_server(timeout = rospy.Duration()):
            msg = 'Joint trajectory action server for {0} not found'.format(self.name)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        # Suscriber joint state
        self.state = JointState()
        self.state.name = self.joint_names
        self.state.position = [0.0]*Arm.num_joints
        self.state_sub = rospy.Subscriber('/bender/joint_states', JointState, self.update_state)
        #Base msg
        self.clear()

    def get_joint_names(self):
        return self.joint_names

    def update_state(self, msg):
        """
        Actualizar posicion de joints
        """
        i = 0
        for j, joint in enumerate(self.joint_names):
            try:
                i = msg.name.index(joint)
            except:
                continue
            self.state.position[j] = msg.position[i]


    def move_joint(self, angles, interval = 3.0, segments = 10):
        """
        Mover brazo en espacio de joints
        @param angles: Lista con 6 angulos de joint, orden deacuerdo a joint_names
        @param interval: Intervalo de tiempo entre cada punto.
        Ej. Mover a home
        arm.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        """
        dt = interval/segments
        t = 0.1
        inter_points = list()
        for i in range(Arm.num_joints):
            inter_points.append(np.linspace(self.state.position[i],angles[i],segments))
        for j in range(segments):
            point = JointTrajectoryPoint()
            point.positions = [joint[j] for joint in inter_points]
            t += dt
            point.time_from_start = rospy.Duration(t)
            self.goal.trajectory.points.append(point)
        #print self.goal
        rospy.loginfo('Sending new goal for {}'.format(self.name))
        self.jta_client.send_goal(self.goal)
        self.clear()

    def move_joint_blind(self, init, fin):
        """
        Mover brazo en espacio de estados predefinidos
        @param init: posicion inicial
        @param goal: posicion final
        Ej. Mover a home
        arm.move_joint_blind('home','pre_1')
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.blind_planner.go(init,fin)
        for i,joint in enumerate(goal.trajectory.joint_names):
            goal.trajectory.joint_names[i] = self.side + joint[1:]
        rospy.loginfo('Sending new goal for {}'.format(self.name))
        self.jta_client.send_goal(goal)

    def move_trajectory(self, points, interval = 0.8):
        """
        Mover brazo en espacio de joints siguiendo trayectoria
        @param angles: Lista de listas con 6 angulos de joint, orden deacuerdo a joint_names
        @param interval: Intervalo de tiempo entre cada punto.
        Ej. Trayectoria simple
        arm.move_trajectory([ [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.5, 0.0, 0.0, 0.0, 0.0, 0.0] ])
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        i=1
        for p in points:
            point = JointTrajectoryPoint()
            point.positions = p
            point.time_from_start = rospy.Duration(i*interval)
            goal.trajectory.points.append(point)
            i=i+1
        rospy.loginfo('Sending new goal for {}'.format(self.name))
        self.jta_client.send_goal(goal)
        self.clear()

    def move_to_point(self, point, joint_names = [''], interval = 0.8):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point.time_from_start = rospy.Duration(interval)
        goal.trajectory.points.append(point)
        rospy.loginfo('Sending new goal for {}'.format(self.name))
        self.jta_client.send_goal(goal)
        self.clear()

    def stop(self):
        self.jta_client.cancel_goal()

    def wait(self, timeout=10.0):
        self.jta_client.wait_for_result(timeout=rospy.Duration(timeout))
        return self.jta_client.get_result()

    def clear(self):
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.joint_names = self.joint_names

    def set_planner(self, planner):
        rospy.logerr('Arm without planning.')
        pass
      
    def set_position_named(self, pos_name):
        rospy.logerr('Arm without planning.')
        pass
        
    def orientate_gripper(self):
        rospy.logerr('Arm without planning.')
        pass

    def go_home(self):
        rospy.logerr('Arm without planning.')
        pass

    def servoing(self, x, y, z):
        rospy.logerr('Arm without planning.')
        pass


class ArmPlanning(Arm):

    simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)

    """
    ArmPlanning clase para el manejo de brazos usando MoveIt!
    Requiere move_group y bender_planning, estos pueden lanzarse usando:
    $ roslaunch bender_arm planning.launch
    """
    def __init__(self, arm_name):
        Arm.__init__(self, arm_name)
        # Clientes para servicios de planning
        try:
            rospy.wait_for_service(self.name + '_planning/set_position', timeout=5)
        except:
            msg = 'Servicio ' + self.name + '_planning/set_position no encontrado'
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)

        self.position_client = rospy.ServiceProxy(self.name + '_planning/set_position', Position)
        self.position_named_client = rospy.ServiceProxy(self.name + '_planning/set_position_named', PositionNamed)
        self.servoing_client = rospy.ServiceProxy(self.name + '_planning/servoing', PositionServoing)
        self.orientation_client = rospy.ServiceProxy(self.name + '_planning/set_orientation', GripperOrientation)
        self.planner_client = rospy.ServiceProxy(self.name + '_planning/set_planner', SetPlannerID)
        self.grasp_client = rospy.ServiceProxy(self.name + '_planning/grasp', Grasp)
        self.joint_client = rospy.ServiceProxy(self.name + '_planning/set_joint', JointTarget)
        self.capamap_grasp_client = rospy.ServiceProxy('/capability_map', CapabilityMapGrasp)
        self.capamap_distance_client = rospy.ServiceProxy('/base_distance', CapabilityMapBestDistance)
        self.acm_object_client = rospy.ServiceProxy('/acm_object', AddObjectACM)
        # Grasp generator client
        rospy.loginfo('Init grasp generator for {0}'.format(self.name))
        self.grasp_gen_client = actionlib.SimpleActionClient(self.name + '_planning/grasp_generator', GraspGeneratorAction)
        rospy.loginfo('Waiting for grasp generator for {0}'.format(self.name))
        # Wait server to start or exit
        if not self.grasp_gen_client.wait_for_server(timeout = rospy.Duration()):
            msg = 'Grasp generator for {0} not found'.format(self.name)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)

        # Pick generator client
        rospy.loginfo('Init pick client for {0}'.format(self.name))
        self.pick_client = actionlib.SimpleActionClient('/pickup', PickupAction)
        rospy.loginfo('Waiting for pick server for {0}'.format(self.name))
        # Wait server to start or exit
        if not self.pick_client.wait_for_server(timeout = rospy.Duration()):
            msg = 'Pick server for {0} not found'.format(self.name)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)

    def generate_grasp(self, obj, axial_res = 5, angle_res = 10):
        grasp_goal = GraspGeneratorGoal()
        grasp_goal.target = obj
        grasp_goal.axial_resolution = axial_res
        grasp_goal.angle_resolution = angle_res
        self.grasp_gen_client.send_goal(grasp_goal)

    def get_grasp(self, timeout=0.0):
        self.grasp_gen_client.wait_for_result(timeout=rospy.Duration(timeout))
        return self.grasp_gen_client.get_result()

    def pick(self, object_name, grasp, support_name = '', planner = 'BKPIECEkConfigDefault'):
        goal = PickupGoal()
        goal.target_name = object_name
        goal.group_name = self.name
        goal.end_effector = '{0}_gripper'.format(self.side)
        goal.allowed_planning_time = 50
        goal.support_surface_name = support_name # @TODO Anadir este parametro con info del shelfdetector
        goal.planner_id = planner
        goal.allow_gripper_support_collision = True
        goal.attached_object_touch_links = ['bender/l_int_finger_link','bender/l_ext_finger_link','bender/l_wrist_pitch_link']
        goal.allowed_touch_objects = ['pringles']
        #goal.path_constraints = path_constraints
        goal.possible_grasps = grasp

        self.pick_client.send_goal(goal)

    def get_grasp_capmap(self, obj):
        """
        Obtiene grasp precalculados usando Capability Map
        @param obj: Objeto cilindrico, moveit_msg/CollisionObject

        Retorna dict{'pregrasp':[...], 'grasp':[..]}
        """
        target = copy.deepcopy(obj)
        # Parche para lado, por defecto cap_map es para l_arm
        if self.side == 'r':
            for i, pose in enumerate(target.primitive_poses):
                pose.position.y *= -1
                target.primitive_poses[i] = pose
        try:
            rospy.loginfo("Calling Capability Map")
            resp = self.capamap_grasp_client(target, True, self.name)
            if not resp.grasps:
                rospy.logerr('No grasp generated!')
                return None
            result = dict()
            result['pregrasp'] = list()
            result['grasp'] = list()
            for gp in resp.grasps:
                # @TODO Revisar orden, capability map los retorna alreves
                result['pregrasp'].extend([pg.positions for pg in gp.grasp])
                result['grasp'].extend([g.positions for g in gp.pregrasp])
            # Filtro de posicion elbow > 0
            bad_grasp = list()
            for i, pos in enumerate(result['pregrasp']):
                if pos[3] < 0:
                    rospy.logwarn('Filter elbow bad position')
                    bad_grasp.append(i)
            # Borrar elementos
            if bad_grasp:
                rospy.logwarn('Filter elbow bad position')
                for j in reversed(bad_grasp):
                    try:
                        del result['pregrasp'][j]
                        del result['grasp'][j]
                    except:
                        pass
            # Check largo
            if not result['pregrasp']:
                return None
            return result
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return None


    def add_obj_acm(self, obj, touch_links = None):
        # Links del gripper pueden tocar el objeto (octomap)
        default_touch_link = ['wrist_pitch_link', 'int_finger_link', 'ext_finger_link']
        if touch_links is None: #Permite usar listas vacias
            touch_links = ["bender/" + self.side + "_" + link for link in default_touch_link]
        
        try:
            rospy.loginfo('Adding collision object {} (op. {}) with ACM links: {}'.format(obj.id, obj.operation,touch_links))
            self.acm_object_client(obj, touch_links)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def get_distance_capmap(self, obj, range_width = 0.5):
        """
        Obtiene el cambio en la posicion x de la base que maximiza el numero de grasps
        usando Capability Map
        @param obj: Objeto cilindrico, moveit_msg/CollisionObject
        @param range: Rango en x para busqueda

        Retorna distance(double): distancia en x
        """
        target = copy.deepcopy(obj)
        # Parche para lado derecho, por defecto cap_map es para l_arm
        if self.side == 'r':
            for i, pose in enumerate(target.primitive_poses):
                pose.position.y *= -1
                target.primitive_poses[i] = pose
        try:
            rospy.loginfo("Calling Capability Map for distance")
            resp = self.capamap_distance_client(target, range_width)
            return resp.distance
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return 0.0


    def grasp(self, obj):
        """
        Prueba distintas orientaciones y posiciones para un objeto cilindrico
        @param angles: Objeto cilindrico, bender_msg/CylindricalObject

        Retorna un GraspResponse
        """
        try:
            resp = self.grasp_client(obj)
            rospy.loginfo("Setting grasp!")
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return None

    def set_joint(self, joint):
        try:
            resp = self.joint_client(joint)
            rospy.loginfo("Joint target " + str(joint))
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def set_planner(self, planner="LBKPIECEkConfigDefault"):
        try:
            resp = self.planner_client(planner)
            rospy.loginfo("Setting planner id to: " + planner)
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def set_position_named(self, pos_name):
        rospy.loginfo('Sending position named: {}'.format(pos_name))
        try:
            resp = self.position_named_client(pos_name)
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def set_position(self, target, approx_ik = False):
        rospy.loginfo('Sending to [{:.2f}, {:.2f}, {:.2f}] frame_id: {}. Using approx. IK: {}'.format(target.pose.position.x,
            target.pose.position.y, target.pose.position.z, target.header.frame_id, approx_ik))
        target.pose.orientation=Limb.simple_orientation
        req = PositionRequest(target,approx_ik)
        try:
            resp = self.position_client(req)
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
        
    def orientate_gripper(self):
        rospy.loginfo('Orientate gripper')
        target = GripperOrientationRequest()
        target.wrt_base = True
        try:
            resp = self.orientation_client(target)
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def go_home(self):
        self.set_position_named('home')

    def servoing(self, x = 0, y = 0, z = 0):
        rospy.loginfo('Sending servoing with [{:.2f}, {:.2f}, {:.2f}]'.format(x,y,z))
        target = PositionServoingRequest()
        target.delta.position = Point(x,y,z)
        try:
            resp = self.servoing_client(target)
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
    

class Gripper(object):
    def __init__(self, gripper):
        self.name = gripper
        ns = '/bender/' + self.name + '_controller'
        rospy.loginfo('Init GripperClient con %s' % ns + '/gripper_action')
        self.client = actionlib.SimpleActionClient(ns + '/gripper_action', GripperCommandAction)
        self.goal = GripperCommandGoal()
        rospy.loginfo('Waiting for Gripper action server for {0}'.format(self.name))
        # Wait 5 Seconds for the gripper action server to start or exit
        if not self.client.wait_for_server(timeout = rospy.Duration(5.0)):
            msg = 'Gripper action server for {0} not found'.format(self.name)
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        # Base msg
        self.clear()

    def command(self, position, effort=0.5):
        self.goal.command.position = position
        self.goal.command.max_effort = effort
        rospy.loginfo('Sending new goal with position={:.2f} effort={:.2f}'.format(position,effort))
        self.client.send_goal(self.goal)

    def close(self, effort = 0.3):
        rospy.loginfo('Closing gripper...')
        self.command(0.0, effort)
        return self.wait()

    def open(self, effort = 0.8):
        rospy.loginfo('Opening gripper...')
        self.command(0.6, effort)
        return self.wait()

    def stop(self):
        rospy.logwarn('Sending cancel goal...')
        self.client.cancel_goal()

    def wait(self, timeout=10.0):
        self.client.wait_for_result(timeout=rospy.Duration(timeout))
        return self.client.get_result()

    def clear(self):
        self.goal = GripperCommandGoal()

class Limb(object):
    simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)

    def __init__(self, side, enable_planning = True):
        self.side = side
        self.planning = enable_planning
        if (enable_planning):
            self.arm = ArmPlanning(self.side + '_arm')
        else:
            self.arm = Arm(self.side + '_arm')
        self.gripper = Gripper(self.side + '_gripper')

    def check_planning(self):
        if self.planning:
            return True
        else:
            return False
