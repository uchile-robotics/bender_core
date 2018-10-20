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
from std_msgs.msg import Float64

from bender_skills import robot_factory

class JoystickBase(object):


    def __init__(self):

        # loading robot
        self.robot = robot_factory.build(["neck","face","tts","r_arm"],core=False)
        self.neck = self.robot.get("neck")
        self.face = self.robot.get("face")
        self.tts=self.robot.get("tts")
        self.arm=self.robot.get("r_arm")

        rospy.loginfo('Joystick base init ...')

        # connections
        self.pub = rospy.Publisher('base/cmd_vel', Twist, queue_size=1)
        #self.pub_priority = rospy.Publisher('base/master_cmd_vel', Twist, queue_size=1)
        self.pub_priority = rospy.Publisher('/bender/nav/base/cmd_vel', Twist, queue_size=1)
        self.cancel_goal_client = rospy.ServiceProxy('/bender/nav/goal_server/cancel', Empty)
        #R_Arm publishers
        self.rElbowPitch = rospy.Publisher('/bender/r_elbow_pitch_controller/command',Float64, queue_size=1)
        self.rElbowYaw = rospy.Publisher('/bender/r_elbow_yaw_controller/command',Float64, queue_size=1)
        self.rShoulderPitch = rospy.Publisher('/bender/r_shoulder_pitch_controller/command',Float64, queue_size=1)
        self.rShoulderYaw = rospy.Publisher('/bender/r_shoulder_yaw_controller/command',Float64, queue_size=1)
        self.rShoulderRoll = rospy.Publisher('/bender/r_shoulder_roll_controller/command',Float64, queue_size=1)
        self.rWristPitch = rospy.Publisher('/bender/r_wrist_pitch_controller/command',Float64, queue_size=1)

        # control
        self.is_paused = False

        # load configuration
        self.b_pause    = rospy.get_param('~b_pause', 'START')
        self.b_cancel   = rospy.get_param('~b_cancel', 'LS')  # modified
        self.b_priority = rospy.get_param('~b_priority', 'LB')
        self.b_arm = rospy.get_param('~b_arm', 'RB')
        self.b_movetest = rospy.get_param('~b_movetest', 'A') # test
        # head buttons
        self.b_neck_home = rospy.get_param('~b_neck_home','RS')
        self.b_happy = rospy.get_param('~b_happy','A')
        self.b_angry = rospy.get_param('~b_angry','B')
        self.b_sad = rospy.get_param('~b_sad','X')
        self.b_surprise = rospy.get_param('~b_surprise','Y')

        a_linear   = rospy.get_param('~a_linear', 'LS_VERT')
        a_angular  = rospy.get_param('~a_angular', 'LS_HORZ')

        # head analogs
        a_neck_x   = rospy.get_param('~a_neck_x', 'RS_HORZ')
        a_neck_y   = rospy.get_param('~a_neck_y', 'RS_VERT')

        self.max_linear_vel  = rospy.get_param('~max_linear_vel', 0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.5)

        key_mapper = xbox.KeyMapper()
        self.b_idx_pause    = key_mapper.get_button_id(self.b_pause)
        self.b_idx_cancel   = key_mapper.get_button_id(self.b_cancel)
        self.b_idx_priority = key_mapper.get_button_id(self.b_priority)
        self.b_idx_arm = key_mapper.get_button_id(self.b_arm)
        self.b_idx_movetest = key_mapper.get_button_id(self.b_movetest)
        self.a_idx_linear   = key_mapper.get_axis_id(a_linear)
        self.a_idx_angular  = key_mapper.get_axis_id(a_angular)

        # head mappings
        self.b_idx_neck_home    = key_mapper.get_button_id(self.b_neck_home)
        self.a_idx_neck_x       = key_mapper.get_axis_id(a_neck_x)
        self.a_idx_neck_y       = key_mapper.get_axis_id(a_neck_y)

        # emotion mapping
        self.b_idx_happy        = key_mapper.get_button_id(self.b_happy)
        self.b_idx_angry        = key_mapper.get_button_id(self.b_angry)
        self.b_idx_sad          = key_mapper.get_button_id(self.b_sad)
        self.b_idx_surprise     = key_mapper.get_button_id(self.b_surprise)

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

    def armHome(self):
        comm=Float64(0.0)
        self.rElbowYaw.publish(comm)
        self.rElbowPitch.publish(comm)
        self.rShoulderYaw.publish(comm)
        self.rShoulderPitch.publish(comm)
        self.rShoulderRoll.publish(comm)
        self.rWristPitch.publish(comm)

    def moveArm(self, angle):
        self.rElbowYaw.publish(angle[0])
        self.rElbowPitch.publish(angle[1])
        self.rShoulderYaw.publish(angle[2])
        self.rShoulderPitch.publish(angle[3])
        self.rShoulderRoll.publish(angle[4])
        self.rWristPitch.publish(angle[5])


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

            neck_to_left    = msg.axes[self.a_idx_neck_x] > 0.9
            neck_to_right   = msg.axes[self.a_idx_neck_x] < -0.9
            neck_to_up      = msg.axes[self.a_idx_neck_y] > 0.9
            neck_to_down    = msg.axes[self.a_idx_neck_y] < -0.9
            neck_to_home    = msg.buttons[self.b_idx_neck_home] == 1.0

            bender_happy    = msg.buttons[self.b_idx_happy] == 1.0
            bender_angry    = msg.buttons[self.b_idx_angry] == 1.0
            bender_sad      = msg.buttons[self.b_idx_sad] == 1.0
            bender_surprise = msg.buttons[self.b_idx_surprise] == 1.0

            bender_arm      = msg.buttons[self.b_idx_arm] == 1.0

            tts1            = msg.axes[6]== 1.0
            tts2            = msg.axes[6]== -1.0
            tts3            = msg.axes[7]== 1.0
            tts4            = msg.axes[7]== -1.0


            if neck_to_left:
                self.neck.look_left()
                rospy.loginfo("Moving neck to left <---")
            if neck_to_right:
                self.neck.look_right()
                rospy.loginfo("Moving neck to right --->")
            if neck_to_down:
                self.neck.look_at_ground()
                rospy.loginfo("Moving neck to ground ___")
            if neck_to_home:
                self.neck.home()
                rospy.loginfo("Moving to home position")

            if bender_arm :
                
                rospy.logwarn_throttle(2, "Moving Arm")
                if bender_happy:
                    self.arm.send_joint_goal([0.0]*6)
                    rospy.loginfo("Moving to home...")
                if bender_angry:
                    pos_1=[-0.5, 0.0, 0.0, 1.5, 0.0, 0.8]
                    self.arm.send_joint_goal(pos_1)
                    rospy.loginfo("Moving to pre_1...")
                if bender_sad:

                    rospy.loginfo("Moving to pre_2...")
                if bender_surprise:
                    pos_2=[0.1, 0.0, 0.0, 1.1624, 0.0, 0.2565]
                    self.arm.send_joint_goal(pos_2)
                    rospy.loginfo("Moving to home...")
                return

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
                self.face.set_emotion("fear")
                rospy.loginfo("Bender is frightened :O")

            if msg.buttons[self.b_idx_priority]:
                rospy.logwarn_throttle(2, "Drive with care. Using the high priority joystick topic.")
                self.pub_priority.publish(cmd)
            else:
                self.pub.publish(cmd)

            

            if tts1:
                self.tts.say("Hola Amigos")
                rospy.loginfo("Speaking")

            if tts2:
                self.tts.say("omae wa mou chindeiru")
                rospy.loginfo("Speaking")

            if tts3:
                self.tts.say("Los voy a destruir a todos")
                rospy.loginfo("Speaking")

            if tts4:
                self.tts.say("Bienvenidos al festival")
                rospy.loginfo("Speaking")

            if neck_to_up:
                self.tts.say("Hola mi nombre es Bender")
                rospy.loginfo("Speaking")



if __name__ == '__main__':
    rospy.init_node('joy_base')
    JoystickBase()
    rospy.spin()
