#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'


import rospy
import sys
from random import random
from threading import Thread
from tf import transformations
# Msgs
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64
from std_srvs.srv import Empty

# Markers
from visualization_msgs.msg import InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

# Octomap
from bender_arm_planning.srv import ManageOctomap, ManageOctomapRequest
from bender_arm_planning.msg import OctomapOptions

# Arm commander
from bender_arm_control.arm_commander import Limb
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

def get_collision_box(pose, name, frame_id='bender/base_link', dim=[0.1,0.1,0.1]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = frame_id
    obj.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = dim

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD
    return obj

def get_collision_cylinder(pose, name, frame_id='bender/base_link', dim = [0.22, 0.04]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = frame_id
    obj.id = name

    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = dim

    obj.primitives.append(cylinder)
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD
    return obj


class SimpleGrasp(object):
    """Secuencia de grasp simple"""
    def __init__(self):
        rospy.logwarn('Init arm interface')
        self.l_limb = Limb('l')
        self.r_limb = Limb('r')

        # Servicio dummy para limpiar objetos
        rospy.wait_for_service('/l_arm_planning/clear_collision')
        self.clear_objects = rospy.ServiceProxy('/l_arm_planning/clear_collision', Empty)
        rospy.logwarn('SimpleGrasp [OK]')

    def clear_collision(self):
        self.clear_objects()

    def get_limb(self, pose):
        if pose.position.y >= 0:
            return self.l_limb
        else:
            return self.r_limb

    def go_home(self):
        self.l_limb.gripper.close()
        self.r_limb.gripper.close()
        rospy.sleep(1.0)
        self.l_limb.arm.move_joint([0.0]*6)
        self.r_limb.arm.move_joint([0.0]*6)

    # Anadir objetos al acm
    def add_acm(self, pose, obj, touch_links = None):
        limb = self.get_limb(pose)
        limb.arm.add_obj_acm(obj, touch_links)

    def grasp(self, pose, radius, height, frame_id = 'base_link', name = 'pringles', axial_res = 6, angle_res = 10):
        # Cilindro de colision
        obj = CollisionObject()
        obj.header.stamp = rospy.Time.now()
        obj.header.frame_id = frame_id
        obj.id = name

        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]

        obj.primitives.append(cylinder)
        obj.primitive_poses.append(pose)

        obj.operation = CollisionObject.ADD

        # Grasp
        limb = self.get_limb(pose)

        limb.arm.generate_grasp(obj, axial_res, angle_res)
        possible_grasp = limb.arm.get_grasp()

        limb.arm.set_position_named('home')
        rospy.sleep(2.0)
        limb.arm.set_position_named('premanip_1')
        rospy.sleep(3.0)

        if not possible_grasp.ik_solutions:
            rospy.logerr('No se generaron grasps')
            return

        pregrasp_joints = possible_grasp.ik_solutions[2*possible_grasp.order[0]].positions
        grasp_joints = possible_grasp.ik_solutions[2*possible_grasp.order[0]+1].positions
        
        result = limb.arm.set_joint(pregrasp_joints) # Movimiento con planificador

        if (result.error_code.val == MoveItErrorCodes.SUCCESS):
            limb.gripper.open(effort = 300)
            rospy.sleep(1.0)

            limb.arm.move_joint(grasp_joints, interval = 2.5) # Movimiento con collisiones permitidas
            limb.arm.wait()
            rospy.sleep(0.5)

            limb.gripper.close(effort = 300)
            rospy.sleep(1.0)
        else:
            rospy.sleep(2.0)
            limb.arm.set_position_named('premanip_1')
            rospy.sleep(2.0)

    def test_capmap(self, pose, obj):
        # Cilindro de colision
        rospy.loginfo('Target pose [{:.2f}, {:.2f}, {:.2f}]'.format(pose.position.x, pose.position.y, pose.position.z))
        
        # Grasp
        limb = self.get_limb(pose)
        possible_grasp = limb.arm.get_grasp_capmap(obj)
        
        if not possible_grasp:
            rospy.logerr('No se encontraron grasps')
            return
        rospy.loginfo('Se encontraron {} grasps'.format(len(possible_grasp['pregrasp'])))    

    # Accion de grasp usando capability map
    def grasp_capmap(self, pose, obj):
        # Cilindro de colision
        rospy.loginfo('Target pose [{:.2f}, {:.2f}, {:.2f}]'.format(pose.position.x, pose.position.y, pose.position.z))
        
        # Seleccionar brazo usando coordenada y
        limb = self.get_limb(pose)

        # Obtener grasps usando capability map, retorna un diccionario (ver arm_commander)
        # Estas posicioes de pregrasp son filtradas por colision
        # Puede darse el caso que la posicion de pregrasp sea desechada por estar en colision
        # con elementos del octomap  que representan al objeto, para evitar esto se deben reemplazar
        # las celdas del octomap por un objeto de colision que represente de mejor manera la geometria
        # del objeto. Esto se puede hacer con la funcion add_obj_acm(obj) de arm_commander
        # que anade el objeto como objeto de colision pero permite que los link del gripper puedan chocar
        # con el (ie se anaden a la matriz de colisiones permitidas, allowed col. matrix )
        possible_grasp = limb.arm.get_grasp_capmap(obj)

        # Obtener distancia (desplazamiento en x cr base_link) que maximiza el numero de grasp 
        # en intervalo [obj.pose.x-range_width/2, obj.pose.x+range_width/2]
        # Eg. best_distance = 0.1 si el robot se mueve 10cm hacia adelante (+x) se maximiza el
        # el numero grasps sobre el obj
        best_distance = limb.arm.get_distance_capmap(obj, range_width = 0.5)
        rospy.logwarn('Base best distance ' + str(best_distance))
        
        if not possible_grasp:
            rospy.logerr('No se encontraron grasps')
            return

        rospy.loginfo('Se encontraron {} grasps'.format(len(possible_grasp['pregrasp'])))


        # Escoger grasps
        # Los grasps estan ordenados de mayor a menor altura, si la incerteza es alta
        # se recomienda generar grasps con objetos de menor altura y aumentarla si no
        # se encuentran grasps
        pregrasp_joints = possible_grasp['pregrasp'][0]
        grasp_joints = possible_grasp['grasp'][0]

        # Movimientos de brazo y gripper
        limb.gripper.close()
        limb.arm.move_joint_blind('home','pre_1')
        limb.arm.wait()
        
        # Movimiento con planificador
        # Chequea colisiones con octomap y objetos de colision, se excluyen los objetos de la ACM
        result = limb.arm.set_joint(pregrasp_joints)

        limb.gripper.open()
        print "Codigo de error: " + str(result.error_code.val)
        if (result.error_code.val == MoveItErrorCodes.SUCCESS):
            rospy.loginfo('Pregrasp position reached!')
            # Movimiento con collisiones permitidas
            # Usa comunicacion directa con los controladores, puede chocar con cualquier cosa
            limb.arm.move_joint(grasp_joints, interval = 1.5, segments=5) 
            limb.arm.wait()
            res_gripper = limb.gripper.close()
            print res_gripper
            limb.gripper.open()
            limb.arm.move_joint(pregrasp_joints, interval = 1.5, segments=5) # Movimiento con collisiones permitidas
            limb.arm.wait()
            limb.gripper.close()
            limb.arm.set_position_named('premanip_1')
            limb.arm.move_joint_blind('pre_1','home')
            limb.arm.wait()


class OctomapManager(object):
    """Manejo de octomap"""
    def __init__(self):
        # Obtener servicio
        try:
            rospy.wait_for_service('/manage_octomap', timeout=30)
        except:
            msg = 'Servicio /manage_octomap no encontrado'
            rospy.signal_shutdown(msg)
            sys.exit(1)
        self.octomap = rospy.ServiceProxy('/manage_octomap', ManageOctomap)

    def update(self):
        octomap_opt = OctomapOptions(OctomapOptions.UPDATE)
        self.octomap(octomap_opt)

    def stop(self):
        octomap_opt = OctomapOptions(OctomapOptions.STOP)
        self.octomap(octomap_opt)

    def start(self):
        octomap_opt = OctomapOptions(OctomapOptions.START)
        self.octomap(octomap_opt)

    def clear(self):
        octomap_opt = OctomapOptions(OctomapOptions.CLEAR)
        self.octomap(octomap_opt)

class RGBDController():
    """ Posicionamiento cabeza, solo para gazebo! """
    def __init__(self, rate = 1, init_position = 0.0):
        rospy.logwarn("HeadController only for Gazebo!")
        self.pub = rospy.Publisher("/bender/rgbd_head_controller/command", Float64, queue_size = 5)
        self.rate = rate
        self.current_command = Float64(init_position)
        Thread(target=self.pub_loop).start()

    def set_position(self, command = 0.0):
        self.current_command.data = command

    def pub_loop(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.pub.publish(self.current_command)
            r.sleep()

class InteractiveGrasp(object):
  
    """Realizar grasp de objetos cilindricos usando markers"""

    def __init__(self, topic_name = "interactive_grasp", frame_id = "base_link", arm_name = 'l_arm ',radius = 0.04, height = 0.12, init_position = Point(0,0,0)):
        # Octomap
        self.octomap = OctomapManager()

        # Brazo
        self.arm = SimpleGrasp()

        # Cabeza
        self.head = RGBDController()

        # Interactive Marker
        self.menu_handler = MenuHandler()
        self.server = InteractiveMarkerServer(topic_name)
        self.current_pose = Pose()
        self.current_pose.position = init_position

        # @TODO Soportar multiples markers
        self.menu_handler.insert( "Grasp", callback=self.process_feedback)
        self.menu_handler.insert( "Grasp Cap Map", callback=self.process_feedback)
        self.menu_handler.insert( "Go Home", callback=self.process_feedback)
        self.menu_handler.insert( "Clear objects", callback=self.process_feedback)
        self.menu_handler.insert( "Publish shelf", callback=self.process_feedback)
        self.menu_handler.insert( "Test Cap Map", callback=self.process_feedback)
        self.add_grasp_marker(frame_id, radius, height, init_position)
        self.server.applyChanges()
        self.radius = radius
        self.height = height
        self.frame_id = frame_id

    def scan(self):
        self.octomap.clear()
        self.octomap.stop()
        self.head.set_position(0.0)
        self.octomap.start()
        rospy.sleep(0.5)
        self.octomap.stop()
        
        self.head.set_position(0.3)
        self.octomap.start()
        rospy.sleep(0.5)
        self.octomap.stop()
        
        self.head.set_position(0.6)
        self.octomap.start()
        rospy.sleep(0.5)
        self.octomap.stop()
        
        self.head.set_position(0.9)
        self.octomap.start()
        rospy.sleep(0.5)
        self.octomap.stop()
      
    def process_feedback(self, feedback):
        # Actualizar Pose
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            #rospy.loginfo("Pose update")
            #print feedback.pose
            self.current_pose = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # Grasp
            if feedback.menu_entry_id == 1:
                rospy.loginfo("Update octomap...")
                self.octomap.update()
                rospy.loginfo("Grasp...")
                self.arm.grasp(self.current_pose, self.radius, self.height, self.frame_id)

            # --------------------------------------------------------------
            # Grasp Capability map
            # ------------------------------------------------------------------
            elif feedback.menu_entry_id == 2:
                rospy.loginfo("Grasp with Capability Map...")
                # Anade al objeto al ACM permitiendo colisiones del gripper usando planificador
                # El posible problema es que no solo reemplaza celdas del octomap que estan dentro,
                # sino tambien cercanas pudiendo borrar la superficie de apoyo. En este caso la superficie
                # de apoyo puede publicarse como collision object
                obj = get_collision_cylinder(self.current_pose, 'pringles', self.frame_id, dim = [self.height, self.radius])
                self.arm.add_acm(self.current_pose, obj)
                self.scan()
                self.arm.grasp_capmap(self.current_pose, obj)

            # Home
            elif feedback.menu_entry_id == 3:
                rospy.loginfo("Arm home position...")
                self.arm.go_home()

            # Clear Collision obj
            elif feedback.menu_entry_id == 4:
                rospy.loginfo("Clear collision obj")
                self.arm.clear_collision()

            # Publicar el shelf como collision object (algo que le pedi hace mucho tiempo) permite
            # mejora el desempeno del planificador y reduce problemas con el octomap
            # Los datos del mueble pueden calcularse midiendolo, solo varian la distancia y angulo
            # A veces hay que presionar mas de una vez publish shelf
            elif feedback.menu_entry_id == 5:
                rospy.loginfo("Pubish shelf")
                # Estas dimensiones pueden estimarse a priori midiendo el mueble!
                shelf_distance = 0.55
                shelf_yaw = 0.0
                #z_distance = [0.35, 0.7, 1.05] # Shelf gazebo roslaunch bender_gazebo bender.launch world:=shelf
                z_distance = [0.80] # Table
                #surfaces = list()
                for i,height in enumerate(z_distance):
                    pose = get_pose(x=shelf_distance + 0.25, y=0.0, z=height, yaw=shelf_yaw)
                    #print pose.position.z
                    obj = get_collision_box(pose, 's'+str(i), self.frame_id, [0.5, 1.2, 0.03])
                    self.arm.add_acm(pose, obj, touch_links=[])

            # Test capmap
            elif feedback.menu_entry_id == 6:
                rospy.loginfo("Test capmap")
                obj = get_collision_cylinder(self.current_pose, 'pringles', self.frame_id, dim = [self.height, self.radius])
                self.arm.test_capmap(self.current_pose, obj)
        
        self.server.applyChanges()

    def add_grasp_marker(self, frame_id, radius, height, init_position):
        # Descripcion basica
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.pose.position = init_position
        int_marker.scale = max(radius*2, height) + 0.02

        int_marker.name = "grasp_marker"
        int_marker.description = "Graspable object"

        # Geometria
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = radius*2
        marker.scale.y = radius*2
        marker.scale.z = height
        marker.color.r = random()
        marker.color.g = random()
        marker.color.b = random()
        marker.color.a = 1.0

        # Control 6DOF
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        # Control roll
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "roll"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Movimiento en X
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Control yaw
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "yaw"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Movimiento en Z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Control pitch
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "pitch"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Movimiento en y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.process_feedback)
        self.menu_handler.apply(self.server, int_marker.name)

def main():
    rospy.init_node("interactive_grasp_node")
    ref_frame = rospy.get_param('frame_id', 'bender/base_link')
    InteractiveGrasp(frame_id = ref_frame, init_position = Point(0.50, 0.15, 0.6))
    rospy.spin()

if __name__=="__main__":
    main()
