#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'


import rospy
# Dynamixel msgs
from dynamixel_driver.dynamixel_const import *
from dynamixel_msgs.msg import JointState
from dynamixel_msgs.msg import MotorStateList
from std_msgs.msg import Float64
# Dynamic reconfigure
from bender_arm_control.server import Server as DynamicReconfServer
from bender_arm_control.cfg import MotorMXParamsConfig


class JointPositionMXController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        # Get params from yaml file
        self.joint_name = rospy.get_param(self.controller_namespace + '/joint_name')
        self.joint_speed = rospy.get_param(self.controller_namespace + '/joint_speed', 1.0)
        self.p_gain = rospy.get_param(self.controller_namespace + '/p_gain', None)
        self.i_gain = rospy.get_param(self.controller_namespace + '/i_gain', None)
        self.d_gain = rospy.get_param(self.controller_namespace + '/d_gain', None)
        self.torque_limit = rospy.get_param(self.controller_namespace + '/joint_torque_limit', None)
        self.torque_enable = rospy.get_param(self.controller_namespace + '/torque_enable_init', False)
        # Check params
        self.__ensure_limits()
        # Motor params from yaml
        self.motor_id = rospy.get_param(self.controller_namespace + '/motor/id')
        self.initial_position_raw = rospy.get_param(self.controller_namespace + '/motor/init')
        self.min_angle_raw = rospy.get_param(self.controller_namespace + '/motor/min')
        self.max_angle_raw = rospy.get_param(self.controller_namespace + '/motor/max')
        if rospy.has_param(self.controller_namespace + '/motor/acceleration'):
            self.acceleration = rospy.get_param(self.controller_namespace + '/motor/acceleration')
        else:
            self.acceleration = None
        # Flipped
        self.flipped = self.min_angle_raw > self.max_angle_raw
        # Base joint state msgs
        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])
        # Dynamic reconfigure
        self.reconfig_server = DynamicReconfServer(MotorMXParamsConfig, self.process_params, self.joint_name)
    
    def __ensure_limits(self):
        if self.p_gain is not None:
            if self.p_gain < 0:
                self.p_gain = 0
            elif self.p_gain > 254:
                self.p_gain = 254
            else:
                self.p_gain = int(self.p_gain)
            
        if self.i_gain is not None:
            if self.i_gain < 0:
                self.i_gain = 0
            elif self.i_gain > 254:
                self.i_gain = 254
            else:
                self.i_gain = int(self.i_gain)

        if self.d_gain is not None:
            if self.d_gain < 0:
                self.d_gain = 0
            elif self.d_gain > 254:
                self.d_gain = 254
            else:
                self.d_gain = int(self.d_gain)
            
        if self.torque_limit is not None:
            if self.torque_limit < 0:
                self.torque_limit = 0.0
            elif self.torque_limit > 1:
                self.torque_limit = 1.0

    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState, queue_size=20)
        # Topic for simple command
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Float64, self.process_command)
        # Suscribe to motor states for get joint state
        self.motor_states_sub = rospy.Subscriber('motor_states/%s' % self.port_namespace, MotorStateList, self.process_motor_states)

    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()
    
    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
        if not self.motor_id in available_ids:
            rospy.logwarn('The specified motor id is not connected and responding.')
            rospy.logwarn('Available ids: %s' % str(available_ids))
            rospy.logwarn('Specified id: %d' % self.motor_id)
            return False
            
        self.RADIANS_PER_ENCODER_TICK = rospy.get_param('dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.ENCODER_TICKS_PER_RADIAN = rospy.get_param('dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.motor_id))
        
        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            
        self.ENCODER_RESOLUTION = rospy.get_param('dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.motor_id))
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = rospy.get_param('dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.motor_id))
        self.MAX_VELOCITY = rospy.get_param('dynamixel/%s/%d/max_velocity' % (self.port_namespace, self.motor_id))
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK
        
        if self.p_gain is not None:
            self.set_p_gain(self.p_gain)
        if self.i_gain is not None:
            self.set_i_gain(self.i_gain)
        if self.d_gain is not None:
            self.set_d_gain(self.d_gain)
        
        if self.torque_limit is not None:
            self.set_torque_limit(self.torque_limit)
        if self.acceleration is not None:
            rospy.loginfo("Setting acceleration of %d to %d" % (self.motor_id, self.acceleration))
            self.dxl_io.set_acceleration(self.motor_id, self.acceleration)

        self.joint_max_speed = rospy.get_param(self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY)
        
        if self.joint_max_speed < self.MIN_VELOCITY:
            self.joint_max_speed = self.MIN_VELOCITY
        elif self.joint_max_speed > self.MAX_VELOCITY:
            self.joint_max_speed = self.MAX_VELOCITY
        
        if self.joint_speed < self.MIN_VELOCITY:
            self.joint_speed = self.MIN_VELOCITY
        elif self.joint_speed > self.joint_max_speed:
            self.joint_speed = self.joint_max_speed
        
        self.set_speed(self.joint_speed)
        self.set_torque_enable(self.torque_enable)
        
        return True

    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        #print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * encoder_ticks_per_radian
        #print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick

    def pos_rad_to_raw(self, pos_rad):
        if pos_rad < self.min_angle:
            pos_rad = self.min_angle
        elif pos_rad > self.max_angle:
            pos_rad = self.max_angle
        return self.rad_to_raw(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

    def spd_rad_to_raw(self, spd_rad):
        if spd_rad < self.MIN_VELOCITY:
            spd_rad = self.MIN_VELOCITY
        elif spd_rad > self.joint_max_speed:
            spd_rad = self.joint_max_speed
        # velocity of 0 means maximum, make sure that doesn't happen
        return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK)))

    def set_real_goal(self, goal):
        pass

    def pid_start(self):
        pass

    def pid_stop(self):
        pass

    def pid_convergence(self, threshold = 0.05):
        return True

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id, torque_enable)
        self.dxl_io.set_multi_torque_enabled([mcv])

    def set_speed(self, speed):
        mcv = (self.motor_id, self.spd_rad_to_raw(speed))
        self.dxl_io.set_multi_speed([mcv])

    def set_p_gain(self, gain):
        if gain < 0:
            gain = 0
        elif gain > 254:
            gain = 254
        else:
            gain = int(gain)
        self.dxl_io.set_p_gain(self.motor_id, gain)

    def set_i_gain(self, gain):
        if gain < 0:
            gain = 0
        elif gain > 254:
            gain = 254
        else:
            gain = int(gain)
        self.dxl_io.set_i_gain(self.motor_id, gain)
    
    def set_d_gain(self, gain):
        if gain < 0:
            gain = 0
        elif gain > 254:
            gain = 254
        else:
            gain = int(gain)
        self.dxl_io.set_d_gain(self.motor_id, gain)

    def set_torque_limit(self, max_torque):
        if max_torque > 1:
            max_torque = 1.0   # use all torque motor can provide
        elif max_torque < 0:
            max_torque = 0.0   # turn off motor torque
        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
        mcv = (self.motor_id, raw_torque_val)
        self.dxl_io.set_multi_torque_limit([mcv])

    def update_control_params(self, params):
        self.set_p_gain(params[0])
        self.set_i_gain(params[1])
        self.set_d_gain(params[2])

    def set_acceleration_raw(self, acc):
        if acc < 0:
            acc = 0
        elif acc > 254:
            acc = 254
        self.dxl_io.set_acceleration(self.motor_id, acc)

    def process_motor_states(self, state_list):
        if self.running:
            state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
            if state:
                state = state[0]
                self.joint_state.motor_temps = [state.temperature]
                self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
                self.joint_state.error = state.error * self.RADIANS_PER_ENCODER_TICK
                self.joint_state.velocity = state.speed * self.VELOCITY_PER_TICK
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)

    def process_command(self, msg):
        angle = msg.data
        mcv = (self.motor_id, self.pos_rad_to_raw(angle))
        self.dxl_io.set_multi_position([mcv])

    def process_params(self, config, level):
        if not self.running:
            config['speed'] = self.joint_speed if self.joint_speed is not None else MotorMXParamsConfig.defaults['speed']
            config['p_gain'] = self.p_gain if self.p_gain is not None else MotorMXParamsConfig.defaults['p_gain']
            config['i_gain'] = self.i_gain if self.i_gain is not None else MotorMXParamsConfig.defaults['i_gain']
            config['d_gain'] = self.d_gain if self.d_gain is not None else MotorMXParamsConfig.defaults['d_gain']
            config['torque_limit'] = self.torque_limit if self.torque_limit is not None else MotorMXParamsConfig.defaults['torque_limit']
            config['torque_enable'] = self.torque_enable
            return config

        # Check changes
        if bool(MotorMXParamsConfig.level['speed'] & level):
            rospy.loginfo('speed change')
            self.set_speed(config['speed'])

        if bool(MotorMXParamsConfig.level['p_gain'] & level):
            rospy.loginfo('p_gain change')
            self.set_p_gain(config['p_gain'])

        if bool(MotorMXParamsConfig.level['i_gain'] & level):
            rospy.loginfo('i_gain change')
            self.set_i_gain(config['i_gain'])

        if bool(MotorMXParamsConfig.level['d_gain'] & level):
            rospy.loginfo('d_gain change')
            self.set_d_gain(config['d_gain'])

        if bool(MotorMXParamsConfig.level['torque_limit'] & level):
            rospy.loginfo('torque_limit change')
            self.set_torque_limit(config['torque_limit'])

        if bool(MotorMXParamsConfig.level['torque_enable'] & level):
            rospy.loginfo('torque_enable change')
            self.set_torque_enable(config['torque_enable'])

        return config