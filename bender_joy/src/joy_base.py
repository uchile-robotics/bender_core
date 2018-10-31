#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped, Point  # added
from bender_joy import xbox
from moveit_python import MoveGroupInterface            # added

from bender_skills import robot_factory

class JoystickBase(object):

    def __init__(self):

        # loading robot
        self.robot  = robot_factory.build(["neck","face"],core=False)
        # self.robot  = robot_factory.build(["neck","face","tts"],core=False)
        self.neck   = self.robot.get("neck")
        self.face   = self.robot.get("face")
        # self.tts    = self.robot.get("tts")

        # tts config
        # self.tts.set_language("spanish")

        self.text_tts1 = "Frase de TTS 1"
        self.text_tts2 = "Frase de TTS 2"
        self.text_tts3 = "Frase de TTS 3"
        self.text_tts4 = "Frase de TTS 4"

        rospy.loginfo('Joystick base init ...')

        # connections
        self.pub = rospy.Publisher('base/cmd_vel', Twist, queue_size=1)
        self.pub_priority = rospy.Publisher('base/master_cmd_vel', Twist, queue_size=1)
        self.cancel_goal_client = rospy.ServiceProxy('/bender/nav/goal_server/cancel', Empty)

        # control
        self.is_paused = False

        # load configuration
        self.b_pause    = rospy.get_param('~b_pause', 'START')
        self.b_cancel   = rospy.get_param('~b_cancel', 'LS')  # modified
        self.b_priority = rospy.get_param('~b_priority', 'LB')
        # head buttons
        self.b_neck_send    = rospy.get_param('~b_neck_send','RS')
        self.b_happy        = rospy.get_param('~b_happy','A')
        self.b_angry        = rospy.get_param('~b_angry','B')
        self.b_sad          = rospy.get_param('~b_sad','X')
        self.b_surprise     = rospy.get_param('~b_surprise','Y')

        a_linear   = rospy.get_param('~a_linear', 'LS_VERT')
        a_angular  = rospy.get_param('~a_angular', 'LS_HORZ')

        # debug triggers
        self.a_debug_l = rospy.get_param('~a_debug_l', 'LT')
        self.a_debug_r = rospy.get_param('~a_debug_r', 'RT')

        # head analogs
        self.a_neck_x   = rospy.get_param('~a_neck_x', 'RS_HORZ')
        self.a_neck_y   = rospy.get_param('~a_neck_y', 'RS_VERT')

        # tts buttons
        self.b_tts1 = rospy.get_param('~b_tts1', 'UP')
        self.b_tts2 = rospy.get_param('~b_tts1', 'LEFT')
        self.b_tts3 = rospy.get_param('~b_tts1', 'DOWN')
        self.b_tts4 = rospy.get_param('~b_tts1', 'RIGHT')

        # limit parameters
        self.max_yaw_pos        = rospy.get_param('~max_yaw_pos', 1.8)
        self.min_yaw_pos        = rospy.get_param('~min_yaw_pos', -1.8)
        self.home_yaw_pos       = rospy.get_param('~home_yaw_pos', 0.0)
        self.max_pitch_pos      = rospy.get_param('~max_pitch_pos', 0.88)
        self.min_pitch_pos      = rospy.get_param('~min_pitch_pos', -0.39)
        self.home_pitch_pos     = rospy.get_param('~home_pitch_pos', 0.0)
        self.max_linear_vel     = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel    = rospy.get_param('~max_angular_vel', 0.5)

        key_mapper = xbox.KeyMapper()
        self.b_idx_pause    = key_mapper.get_button_id(self.b_pause)
        self.b_idx_cancel   = key_mapper.get_button_id(self.b_cancel)
        self.b_idx_priority = key_mapper.get_button_id(self.b_priority)
        self.a_idx_linear   = key_mapper.get_axis_id(a_linear)
        self.a_idx_angular  = key_mapper.get_axis_id(a_angular)

        # head mappings
        self.b_idx_neck_send    = key_mapper.get_button_id(self.b_neck_send)
        self.a_idx_neck_x       = key_mapper.get_axis_id(self.a_neck_x)
        self.a_idx_neck_y       = key_mapper.get_axis_id(self.a_neck_y)

        # emotion mapping
        self.b_idx_happy        = key_mapper.get_button_id(self.b_happy)
        self.b_idx_angry        = key_mapper.get_button_id(self.b_angry)
        self.b_idx_sad          = key_mapper.get_button_id(self.b_sad)
        self.b_idx_surprise     = key_mapper.get_button_id(self.b_surprise)

        # debug mapping
        self.a_idx_debug_l  = key_mapper.get_axis_id(self.a_debug_l)
        self.a_idx_debug_r  = key_mapper.get_axis_id(self.a_debug_r)

        # tts mapping
        self.b_idx_tts1 = key_mapper.get_button_id(self.b_tts1)
        self.b_idx_tts2 = key_mapper.get_button_id(self.b_tts2)
        self.b_idx_tts3 = key_mapper.get_button_id(self.b_tts3)
        self.b_idx_tts4 = key_mapper.get_button_id(self.b_tts4)

        # check
        self.assert_params()

        # ready to work
        rospy.Subscriber('joy', Joy, self.callback, queue_size=1)
        rospy.loginfo('Joystick for base is ready')

    def assert_params(self):
        """
        checks whether the user parameters are valid or no.
        """
        assert isinstance(self.b_idx_pause, int)
        assert isinstance(self.a_idx_angular, int)
        assert isinstance(self.a_idx_linear, int)

    # this method breaks the decoupling between base and soft ws!!!
    def cancel_goal(self):
        try:
            self.cancel_goal_client.wait_for_service(0.5)
            self.cancel_goal_client()
            rospy.loginfo("Goal cancelled")
        except rospy.ServiceException:
            rospy.loginfo("There is no goal to cancel")
        except Exception:
            pass

    def callback(self, msg):

        # pause
        if msg.buttons[self.b_idx_pause]:

            self.is_paused = not self.is_paused
            if self.is_paused:

                # stop signal
                cmd = Twist()
                self.pub.publish(cmd)

                rospy.logwarn("\nControlling PAUSED!, press the " + self.b_pause + " button to resume it\n")
            else:
                rospy.logwarn("Controlling RESUMED, press " + self.b_pause + " button to pause it")

            # very important sleep!
            # prevents multiple triggersfor the same button
            rospy.sleep(1)  # it should be >= 1;
            return

        elif msg.buttons[self.b_idx_cancel]:
            self.cancel_goal()
            return

        # work
        if not self.is_paused:

            cmd = Twist()
            cmd.angular.z = self.max_angular_vel * msg.axes[self.a_idx_angular]
            cmd.linear.x = self.max_linear_vel * msg.axes[self.a_idx_linear]

            yaw_neck    = 0.0
            pitch_neck  = 0.0

            neck_to_left    = msg.axes[self.a_idx_neck_x] > 0.0
            neck_to_right   = msg.axes[self.a_idx_neck_x] < 0.0
            neck_to_up      = msg.axes[self.a_idx_neck_y] > 0.0
            neck_to_down    = msg.axes[self.a_idx_neck_y] < 0.0

            neck_confirmed  = msg.buttons[self.b_idx_neck_send] == 1.0

            speak_tts1      = msg.buttons[self.b_idx_tts1] == 1.0
            speak_tts2      = msg.buttons[self.b_idx_tts2] == 1.0
            speak_tts3      = msg.buttons[self.b_idx_tts3] == 1.0
            speak_tts4      = msg.buttons[self.b_idx_tts4] == 1.0

            bender_happy    = msg.buttons[self.b_idx_happy] == 1.0
            bender_angry    = msg.buttons[self.b_idx_angry] == 1.0
            bender_sad      = msg.buttons[self.b_idx_sad] == 1.0
            bender_surprise = msg.buttons[self.b_idx_surprise] == 1.0

            left_triggered  = msg.axes[self.a_idx_debug_l] == -1.0
            right_triggered = msg.axes[self.a_idx_debug_r] == -1.0
            debugging       = left_triggered and right_triggered

            if neck_confirmed:
                if neck_to_left:    yaw_neck = msg.axes[self.a_idx_neck_x] * abs(self.max_yaw_pos)
                if neck_to_right:   yaw_neck = msg.axes[self.a_idx_neck_x] * abs(self.min_yaw_pos)
                if neck_to_up:      pitch_neck = - msg.axes[self.a_idx_neck_y] * abs(self.min_pitch_pos)
                if neck_to_down:    pitch_neck = - msg.axes[self.a_idx_neck_y] * abs(self.max_pitch_pos)
                self.neck.stop
                self.neck.send_joint_goal(yaw_neck,pitch_neck)

            if debugging: rospy.loginfo("Debugging the debugger")

            if speak_tts1: rospy.loginfo("TTS1 pressed") # self.tts.say(self.text_tts1)
            if speak_tts2: rospy.loginfo("TTS2 pressed") # self.tts.say(self.text_tts2)
            if speak_tts3: rospy.loginfo("TTS3 pressed") # self.tts.say(self.text_tts3)
            if speak_tts4: rospy.loginfo("TTS4 pressed") # self.tts.say(self.text_tts4)

            if bender_happy:
                self.face.set_emotion("happy1")
                rospy.loginfo("Bender is happy :D")
            if bender_angry:
                self.face.set_emotion("angry1")
                rospy.loginfo("Bender is angry >:(")
            if bender_sad:
                self.face.set_emotion("sad1")
                rospy.loginfo("Bender is sad :c")
            if bender_surprise:
                self.face.set_emotion("surprise")
                rospy.loginfo("Bender is surprised :O")

            if msg.buttons[self.b_idx_priority]:
                rospy.logwarn_throttle(2, "Drive with care. Using the high priority joystick topic.")
                self.pub_priority.publish(cmd)
            else:
                self.pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('joy_base')
    JoystickBase()
    rospy.spin()
