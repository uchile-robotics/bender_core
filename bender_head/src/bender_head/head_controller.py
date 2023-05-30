#!/usr/bin/env python

# __author__: Gonzalo Olguin
# __email__: golguinm@gmail.com

import roslib
roslib.load_manifest('head_controller')

import rospy 
import os
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from math import pi
import numpy as np

DXL_BAUD_RATE = 1000000
DXL_YAW_ID    = 30
DXL_PITCH_ID  = 31
DXL_PORT      = '/dev/ttyUSB0'

YAW_LIMITS   = [478, 1020]
PITCH_LIMITS = [420,  667]

RAD_TO_DEG = 180.0 / pi
IMAGE_SHAPE = [480, 640]


class HeadController(object):
    def __init__(self, YAW_ID, PITCH_ID, YAW_LIM, PITCH_LIM):
        
        # parameters
        self.dxl_yaw_id   = YAW_ID
        self.dxl_pitch_id = PITCH_ID
        self.yaw_limits   = YAW_LIM
        self.pitch_limits = PITCH_LIM

        self.yaw_pos   = None
        self.pitch_pos = None
        self.joint_states = np.array([self.yaw_pos, self.pitch_pos])

        self.Kp_yaw   = 5
        self.Ki_yaw   = .1
        self.Kd_yaw   = .1
        self.Kp_pitch = 5
        self.Ki_pitch = .1
        self.Kd_pitch = .1

        # timing objects
        self.control_frecuency = 20
        self._control_rate = rospy.Rate(self.control_frecuency)     

        # subscribers
        self._joint_states_sub = rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, self.js_cb)
        self._target_coords_sub = rospy.Subscriber('/detected_face/coords', Int32MultiArray, self.coords_cb)
        
        # services
        self._head_srv = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')

        self.target_x = None
        self.target_y = None

    def js_cb(self, joint_msg):
        self.yaw_pos   = joint_msg.dynamixel_state[1].present_position
        self.pitch_pos = joint_msg.dynamixel_state[0].present_position
        self.joint_states[0] = self.yaw_pos
        self.joint_states[1] = self.pitch_pos
    
    def coords_cb(self, coords_msg):
        self.target_x = coords_msg.data[0]
        self.target_y = coords_msg.data[1]

    def move_to_target_pid(self, dxl_yaw_target, dxl_pitch_target, target_tol=5):
        if dxl_yaw_target > self.yaw_limits[1] or dxl_yaw_target < self.yaw_limits[0]: print(' Yaw out of range')
        if dxl_pitch_target > self.pitch_limits[1] or dxl_pitch_target < self.pitch_limits[0]: print(' Pitch out of range')
        
        print('TARGET: ', dxl_yaw_target, dxl_pitch_target)
        e_i_yaw , e_i_pitch = 0, 0
        e_d = 0
                                                                                                                                                               
        e_yaw   = int(dxl_yaw_target - self.map_value(self.yaw_pos, [-0.1738508939743042, 2.587310552597046], self.yaw_limits))
        e_pitch = int(dxl_pitch_target - self.map_value(self.pitch_pos, [-0.4857598543167114, 0.7976688146591187], self.pitch_limits))
        print(e_yaw, e_pitch)
        while abs(e_yaw) > target_tol or abs(e_pitch) > target_tol:  
            p_yaw = self.map_value(self.yaw_pos, [-0.1738508939743042, 2.587310552597046], self.yaw_limits)
            p_pitch = self.map_value(self.pitch_pos, [-0.4857598543167114, 0.7976688146591187], self.pitch_limits)

            yaw_cmd = p_yaw + e_yaw * self.Kp_yaw + e_i_yaw * (1 / self.control_frecuency) * self.Ki_yaw
            pitch_cmd = p_pitch + e_pitch * self.Kp_pitch + e_i_pitch * (1 / self.control_frecuency) * self.Ki_pitch
            yaw_cmd = self.constrain(yaw_cmd, self.yaw_limits[0], self.yaw_limits[1])
            pitch_cmd = self.constrain(pitch_cmd, self.pitch_limits[0], self.pitch_limits[1])
   
            rospy.loginfo("Control cmd: (%d, %d) | current pos: (%f, %f)", yaw_cmd, pitch_cmd,
             self.map_value(self.yaw_pos, [-0.1738508939743042, 2.587310552597046], self.yaw_limits),
             self.map_value(self.pitch_pos, [-0.4857598543167114, 0.7976688146591187], self.pitch_limits))

            self._head_srv(id=self.dxl_yaw_id, addr_name='Goal_Position', value=yaw_cmd)
            self._head_srv(id=self.dxl_pitch_id, addr_name='Goal_Position', value=pitch_cmd)

            e_yaw   = int(dxl_yaw_target - self.map_value(self.yaw_pos, [-0.1738508939743042, 2.587310552597046], self.yaw_limits))
            e_pitch = int(dxl_pitch_target - self.map_value(self.pitch_pos, [-0.4857598543167114, 0.7976688146591187], self.pitch_limits))

            self._control_rate.sleep()

    def move(self, dxl_yaw_target, dxl_pitch_target):
        # para controlar la velocidda Present_Velocity
        if dxl_yaw_target > self.yaw_limits[1] or dxl_yaw_target < self.yaw_limits[0]: print(' Yaw out of range')
        if dxl_pitch_target > self.pitch_limits[1] or dxl_pitch_target < self.pitch_limits[0]: print(' Pitch out of range')

        self._head_srv(id=self.dxl_yaw_id, addr_name='Goal_Position', value=dxl_yaw_target)
        self._head_srv(id=self.dxl_pitch_id, addr_name='Goal_Position', value=dxl_pitch_target)

    def set_yaw_pitch_speeds(self, speed_yaw, speed_pitch): # speed rpm
        self._head_srv(id=self.dxl_yaw_id,   addr_name='Moving_Speed', value=speed_yaw)
        self._head_srv(id=self.dxl_pitch_id, addr_name='Moving_Speed', value=speed_pitch)

    def set_speed(self, dxl_id, dxl_speed):
        """
        dxl_speed (int32): Moving velocity, 
                           CW:  0 - 1023
                           CCW: 1024 - 2047
        """
        self.constrain(dxl_speed, 0, 2047)
        self._head_srv(id=dxl_id, addr_name='Moving_Speed', value=dxl_speed)

    def set_pos(self, dxl_id, dxl_pos):
        """
        Set position when using position control mode.
        input:
            dxl_id  (int): Dynamixel id
            dxl_pos (int): Hex position
        """

        self._head_srv(id=dxl_id, addr_name='Goal_Position', value=dxl_pos)

    def go_to_pos_velocity(self, dxl_id, dxl_pos):
        """
        Set position when using velocity control mode.
        input:
            dxl_id  (int): Dynamixel id
            dxl_pos (int): Hex position
        """

        js_id = 0 if dxl_id==30 else 1
        error = self.joint_states[js_id] - dxl_pos
        pass
    
    def go_to_joint_pos_velocity(self, dxl_yaw_pos, dxl_pitch_pos):
        """
        Set both motors position when using velocity control mode.
        input:
            dxl_yaw_pos   (int): Dynamixel yaw Hex position
            dxl_pitch_pos (int): Dynamixel pitch Hex position
        """

        e_yaw = self.yaw_pos - dxl_yaw_pos
        e_pitch = dxl_pitch_pos - self.pitch_pos

        while abs(e_yaw) > 10 or abs(e_pitch) > 10:
            yaw_cmd = e_yaw * self.Kp_yaw
            pitch_cmd = e_pitch * self.Kp_pitch
            
            print(e_yaw, e_pitch)

            if yaw_cmd < 0:
                yaw_cmd = 1024 + self.constrain(abs(int(yaw_cmd)), 30, 300)
            else:
                yaw_cmd = self.constrain(int(yaw_cmd), 20, 300)

            if pitch_cmd < 0:
                pitch_cmd = 1024 + self.constrain(abs(int(pitch_cmd)), 20, 300)
            else:
                pitch_cmd = self.constrain(int(pitch_cmd), 20, 300)
            
            e_yaw = dxl_yaw_pos - self.yaw_pos
            e_pitch = dxl_pitch_pos - self.pitch_pos

            self.set_yaw_pitch_speeds(yaw_cmd, pitch_cmd)

        #rospy.loginfo('Target Reached')
        self.set_yaw_pitch_speeds(0, 0)

    @staticmethod
    def constrain(val, pmin, pmax):
        return max(pmin, min(val, pmax))

    @staticmethod
    def map_value(value, r_ori, r_new):
        """
        Map a value from range [a, b] to range [x, y].
        """
        a, b = r_ori
        x, y = r_new
        return (value - a) * (y - x) / (b - a) + x


if __name__=='__main__':
    rospy.init_node('head_controller_node')
    h = HeadController(DXL_YAW_ID, DXL_PITCH_ID, YAW_LIMITS, PITCH_LIMITS)
    while not rospy.is_shutdown():
        x = int(raw_input('value? '))
        y = int(raw_input('value? '))
        h.go_to_joint_pos_velocity(x, y)
        # rospy.sleep(1)
        # rospy.loginfo("Current pos: (%f, %f)",
        #      h.map_value(h.yaw_pos, [-0.1738508939743042, 2.587310552597046], h.yaw_limits),
        #      h.map_value(h.pitch_pos, [-0.4857598543167114, 0.7976688146591187], h.pitch_limits))