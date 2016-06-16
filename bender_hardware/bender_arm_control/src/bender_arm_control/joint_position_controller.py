#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
# Thread
from threading import Thread, Event

from control_util.pid import PID
from dynamixel_driver.dynamixel_const import *

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_msgs.msg import MotorStateList
# Dynamic reconfigure
from bender_arm_control.server import Server as DynamicReconfServer
from bender_arm_control.cfg import MotorParamsConfig

class JointPositionController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        self.joint_name = rospy.get_param(self.controller_namespace + '/joint_name')
        self.joint_speed = rospy.get_param(self.controller_namespace + '/joint_speed', 1.0)
        self.compliance_slope = rospy.get_param(self.controller_namespace + '/joint_compliance_slope', None)
        self.compliance_margin = rospy.get_param(self.controller_namespace + '/joint_compliance_margin', None)
        self.compliance_punch = rospy.get_param(self.controller_namespace + '/joint_compliance_punch', None)
        self.torque_limit = rospy.get_param(self.controller_namespace + '/joint_torque_limit', None)
        self.torque_enable = rospy.get_param(self.controller_namespace + '/torque_enable_init', False)
        
        self.__ensure_limits()
        
        self.motor_id = rospy.get_param(self.controller_namespace + '/motor/id')
        self.initial_position_raw = rospy.get_param(self.controller_namespace + '/motor/init')
        self.min_angle_raw = rospy.get_param(self.controller_namespace + '/motor/min')
        self.max_angle_raw = rospy.get_param(self.controller_namespace + '/motor/max')
        if rospy.has_param(self.controller_namespace + '/motor/acceleration'):
            self.acceleration = rospy.get_param(self.controller_namespace + '/motor/acceleration')
        else:
            self.acceleration = None
        
        self.flipped = self.min_angle_raw > self.max_angle_raw
        
        self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id])
        # PID control loop
        self.enable_pid = rospy.get_param(self.controller_namespace + '/enable_pid', False)
        self.p_gain = rospy.get_param(self.controller_namespace + '/p_gain', 0.0)
        self.i_gain = rospy.get_param(self.controller_namespace + '/i_gain', 0.0)
        self.d_gain = rospy.get_param(self.controller_namespace + '/d_gain', 0.0)
        self.i_clamp = rospy.get_param(self.controller_namespace + '/i_clamp', None)

        print [self.joint_name,self.enable_pid, self.p_gain, self.i_gain, self.d_gain, self.i_clamp]

        self.pid = PID(kp=self.p_gain, ki=self.i_gain, kd=self.d_gain, i_clamp = self.i_clamp)
        self.real_goal = 0.0
        

        # Dynamic reconf server
        self.reconfig_server = DynamicReconfServer(MotorParamsConfig, self.process_params, self.joint_name)


    def __ensure_limits(self):
        if self.compliance_slope is not None:
            if self.compliance_slope < DXL_MIN_COMPLIANCE_SLOPE:
                self.compliance_slope = DXL_MIN_COMPLIANCE_SLOPE
            elif self.compliance_slope > DXL_MAX_COMPLIANCE_SLOPE:
                self.compliance_slope = DXL_MAX_COMPLIANCE_SLOPE
            else:
                self.compliance_slope = int(self.compliance_slope)
            
        if self.compliance_margin is not None:
            if self.compliance_margin < DXL_MIN_COMPLIANCE_MARGIN:
                self.compliance_margin = DXL_MIN_COMPLIANCE_MARGIN
            elif self.compliance_margin > DXL_MAX_COMPLIANCE_MARGIN:
                self.compliance_margin = DXL_MAX_COMPLIANCE_MARGIN
            else:
                self.compliance_margin = int(self.compliance_margin)
            
        if self.compliance_punch is not None:
            if self.compliance_punch < DXL_MIN_PUNCH:
                self.compliance_punch = DXL_MIN_PUNCH
            elif self.compliance_punch > DXL_MAX_PUNCH:
                self.compliance_punch = DXL_MAX_PUNCH
            else:
                self.compliance_punch = int(self.compliance_punch)
            
        if self.torque_limit is not None:
            if self.torque_limit < 0:
                self.torque_limit = 0.0
            elif self.torque_limit > 1:
                self.torque_limit = 1.0


    def start(self):
        self.running = True
        self.joint_state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState, queue_size=20)
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Float64, self.process_command)
        self.motor_states_sub = rospy.Subscriber('motor_states/%s' % self.port_namespace, MotorStateList, self.process_motor_states)
        # pid loop
        self.start_loop = Event()
        self.close_loop = Event()
        self.flag = False
        Thread(target=self.pid_loop).start()


    def stop(self):
        self.running = False
        self.joint_state_pub.unregister()
        self.motor_states_sub.unregister()
        self.command_sub.unregister()

    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        #print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * encoder_ticks_per_radian
        #print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick

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
        
        if self.compliance_slope is not None:
            self.set_compliance_slope(self.compliance_slope)
        if self.compliance_margin is not None:
            self.set_compliance_margin(self.compliance_margin)
        if self.compliance_punch is not None:
            self.set_compliance_punch(self.compliance_punch)
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
        rospy.logdebug('Setting real goal at {:.2f}'.format(goal))
        self.real_goal =  goal

    def set_torque_enable(self, torque_enable):
        mcv = (self.motor_id, torque_enable)
        self.dxl_io.set_multi_torque_enabled([mcv])

    def set_speed(self, speed):
        mcv = (self.motor_id, self.spd_rad_to_raw(speed))
        self.dxl_io.set_multi_speed([mcv])

    def set_compliance_slope(self, slope):
        if slope < DXL_MIN_COMPLIANCE_SLOPE:
            slope = DXL_MIN_COMPLIANCE_SLOPE
        elif slope > DXL_MAX_COMPLIANCE_SLOPE:
            slope = DXL_MAX_COMPLIANCE_SLOPE
        mcv = (self.motor_id, slope, slope)
        self.dxl_io.set_multi_compliance_slopes([mcv])

    def set_compliance_margin(self, margin):
        if margin < DXL_MIN_COMPLIANCE_MARGIN:
            margin = DXL_MIN_COMPLIANCE_MARGIN
        elif margin > DXL_MAX_COMPLIANCE_MARGIN:
            margin = DXL_MAX_COMPLIANCE_MARGIN
        else:
            margin = int(margin)
        mcv = (self.motor_id, margin, margin)
        self.dxl_io.set_multi_compliance_margins([mcv])

    def set_compliance_punch(self, punch):
        if punch < DXL_MIN_PUNCH:
            punch = DXL_MIN_PUNCH
        elif punch > DXL_MAX_PUNCH:
            punch = DXL_MAX_PUNCH
        else:
            punch = int(punch)
        mcv = (self.motor_id, punch)
        self.dxl_io.set_multi_punch([mcv])

    def set_torque_limit(self, max_torque):
        if max_torque > 1:
            max_torque = 1.0         # use all torque motor can provide
        elif max_torque < 0:
            max_torque = 0.0       # turn off motor torque
        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
        mcv = (self.motor_id, raw_torque_val)
        self.dxl_io.set_multi_torque_limit([mcv])

    def update_control_params(self, params):
        self.set_compliance_slope(params[0])
        self.set_compliance_margin(params[1])
        self.set_compliance_punch(params[2])

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
                self.joint_state.error = self.real_goal - self.joint_state.current_pos # new error wrt real_goal state.error * self.RADIANS_PER_ENCODER_TICK
                self.joint_state.velocity = state.speed * self.VELOCITY_PER_TICK
                self.joint_state.load = state.load
                self.joint_state.is_moving = state.moving
                self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
                
                self.joint_state_pub.publish(self.joint_state)

    def pid_loop(self):
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            if self.flag:
                error = self.real_goal - self.joint_state.current_pos
                command = self.real_goal + self.pid.compute_output(error)
                mcv = (self.motor_id, self.pos_rad_to_raw(command))
                try:
                    self.dxl_io.set_multi_position([mcv])
                    #rospy.loginfo('[{0}] Command: {1:.2f}, real {2:.2f}, error {3:.2f}'.format(self.joint_name,command,self.real_goal,self.pid.mean_error()))
                except:
                    rospy.logerr('Error sending position command to {}, disabling PID loop'.format(self.joint_name))
                    self.flag = False
                    self.pid.initialize()
                    
            rate.sleep()

    def pid_start(self):
        self.pid.initialize()
        rospy.logdebug('Init PID for {}'.format(self.joint_name))
        self.flag = True

    def pid_stop(self):
        rospy.logdebug('Stop PID for {}'.format(self.joint_name))
        self.flag = False

    def pid_convergence(self, threshold = 0.1):
        return self.pid.convergence(threshold)

    def process_command(self, msg):
        angle = msg.data
        mcv = (self.motor_id, self.pos_rad_to_raw(angle))
        self.dxl_io.set_multi_position([mcv])

    def process_params(self, config, level):
        if not self.running:
            config['speed'] = self.joint_speed if self.joint_speed is not None else MotorParamsConfig.defaults['speed']
            config['compliance_margin'] = self.compliance_margin if self.compliance_margin is not None else MotorParamsConfig.defaults['compliance_margin']
            config['compliance_slope'] = self.compliance_slope if self.compliance_slope is not None else MotorParamsConfig.defaults['compliance_slope']
            config['punch'] = self.compliance_punch if self.compliance_margin is not None else MotorParamsConfig.defaults['punch']
            config['torque_limit'] = self.torque_limit if self.torque_limit is not None else MotorParamsConfig.defaults['torque_limit']
            config['torque_enable'] = self.torque_enable
            config['p_gain'] = self.p_gain
            config['i_gain'] = self.i_gain
            config['d_gain'] = self.d_gain
            config['i_clamp'] = self.i_clamp
            config['enable_pid'] = self.enable_pid
            return config

        # Check changes
        if bool(MotorParamsConfig.level['speed'] & level):
            rospy.loginfo('speed change')
            self.set_speed(config['speed'])

        if bool(MotorParamsConfig.level['compliance_margin'] & level):
            rospy.loginfo('compliance_margin change')
            self.set_compliance_margin(config['compliance_margin'])

        if bool(MotorParamsConfig.level['compliance_slope'] & level):
            rospy.loginfo('compliance_slope change')
            self.set_compliance_slope(config['compliance_slope'])

        if bool(MotorParamsConfig.level['punch'] & level):
            rospy.loginfo('punch change')
            self.set_compliance_punch(config['punch'])

        if bool(MotorParamsConfig.level['torque_limit'] & level):
            rospy.loginfo('torque_limit change')
            self.set_torque_limit(config['torque_limit'])

        if bool(MotorParamsConfig.level['torque_enable'] & level):
            rospy.loginfo('torque_enable change')
            self.set_torque_enable(config['torque_enable'])

        if bool(MotorParamsConfig.level['p_gain'] & level):
            rospy.loginfo('p_gain change')
            self.pid.set_kp(config['p_gain'])

        if bool(MotorParamsConfig.level['i_gain'] & level):
            rospy.loginfo('i_gain change')
            self.pid.set_ki(config['i_gain'])

        if bool(MotorParamsConfig.level['d_gain'] & level):
            rospy.loginfo('d_gain change')
            self.pid.set_kd(config['d_gain'])

        if bool(MotorParamsConfig.level['i_clamp'] & level):
            rospy.loginfo('i_clamp change')
            self.pid.set_clamp(config['i_clamp'])

        if bool(MotorParamsConfig.level['enable_pid'] & level):
            rospy.loginfo('enable_pid change')
            self.enable_pid = config['enable_pid']

        return config
