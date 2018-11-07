#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Matias Pavez'
__email__ = 'matias.pavez.b@gmail.com'

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from bender_joy import xbox

from bender_skills import robot_factory

class JoystickBase(object):

	def __init__(self):

		# loading robot
		rospy.logwarn("Attemping to build Bender")
		#self.robot      = robot_factory.build(["neck","face","tts","l_arm","r_arm","l_gripper","r_gripper"],core=False)
		self.robot=robot_factory.build(["neck","face","tts","r_arm","r_gripper"],core=False)
		self.neck       = self.robot.get("neck")
		self.face       = self.robot.get("face")
		#self.l_arm      = self.robot.get("l_arm")
		self.r_arm      = self.robot.get("r_arm")
		#self.l_gripper  = self.robot.get("l_gripper")
		self.r_gripper  = self.robot.get("r_gripper")
		self.tts        = self.robot.get("tts")

		# tts config
		self.tts.set_language("spanish")

		self.text_tts1 = "Frase de T T S 1"
		self.text_tts2 = "Frase de T T S 2"
		self.text_tts3 = "Frase de T T S 3"
		self.text_tts4 = "Frase de T T S 4"

		self.r_arm_home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.r_arm_posUp_angles = [0.1, 0.0, 0.0, 1.1624, 0.0, 0.2565]

		rospy.loginfo('Joystick base init ...')

		# connections
		self.pub = rospy.Publisher('base/cmd_vel', Twist, queue_size=1)
		self.pub_priority = rospy.Publisher('base/master_cmd_vel', Twist, queue_size=1)
		self.cancel_goal_client = rospy.ServiceProxy('/bender/nav/goal_server/cancel', Empty)

		# control
		self.is_paused = False

		# load configuration
		self.b_pause    = rospy.get_param('~b_pause', 'START')
		self.b_cancel   = rospy.get_param('~b_cancel', 'LS')
		self.b_priority = rospy.get_param('~b_priority', 'LB')

		# head buttons
		self.b_neck_send    = rospy.get_param('~b_neck_send','RS')
		self.b_a            = rospy.get_param('~b_a','A')
		self.b_b            = rospy.get_param('~b_b','B')
		self.b_x            = rospy.get_param('~b_x','X')
		self.b_y            = rospy.get_param('~b_y','Y')

		# base analogs
		self.a_linear   = rospy.get_param('~a_linear', 'LS_VERT')
		self.a_angular  = rospy.get_param('~a_angular', 'LS_HORZ')

		# debug triggers
		self.a_trigger_l = rospy.get_param('~a_trigger_l', 'LT')
		self.a_trigger_r = rospy.get_param('~a_trigger_r', 'RT')

		# debug buttons
		self.b_trig_l = rospy.get_param('~b_trig_l', 'LB')
		self.b_trig_r = rospy.get_param('~b_trig_r', 'RB')

		# head analogs
		self.a_neck_x   = rospy.get_param('~a_neck_x', 'RS_HORZ')
		self.a_neck_y   = rospy.get_param('~a_neck_y', 'RS_VERT')

		# tts buttons
		self.b_tts1 = rospy.get_param('~b_tts1', 'UP')
		self.b_tts2 = rospy.get_param('~b_tts2', 'LEFT')
		self.b_tts3 = rospy.get_param('~b_tts3', 'DOWN')
		self.b_tts4 = rospy.get_param('~b_tts4', 'RIGHT')

		# hardware limit parameters
		self.max_yaw_pos        = rospy.get_param('~max_yaw_pos', 1.8)
		self.min_yaw_pos        = rospy.get_param('~min_yaw_pos', -1.8)
		self.home_yaw_pos       = rospy.get_param('~home_yaw_pos', 0.0)
		self.max_pitch_pos      = rospy.get_param('~max_pitch_pos', 0.88)
		self.min_pitch_pos      = rospy.get_param('~min_pitch_pos', -0.39)
		self.home_pitch_pos     = rospy.get_param('~home_pitch_pos', 0.0)
		self.max_linear_vel     = rospy.get_param('~max_linear_vel', 0.5)
		self.max_angular_vel    = rospy.get_param('~max_angular_vel', 0.5)

		# demonstration limit parameters
		self.min_yaw_demo   = self.min_yaw_pos * 0.85 / 2
		self.max_yaw_demo   = self.max_yaw_pos * 0.85 / 2
		self.min_pitch_demo = self.min_pitch_pos * 0.80
		self.max_pitch_demo = self.max_pitch_pos * 0.90

		key_mapper = xbox.KeyMapper()
		self.b_idx_pause    = key_mapper.get_button_id(self.b_pause)
		self.b_idx_cancel   = key_mapper.get_button_id(self.b_cancel)
		self.b_idx_priority = key_mapper.get_button_id(self.b_priority)
		self.a_idx_linear   = key_mapper.get_axis_id(self.a_linear)
		self.a_idx_angular  = key_mapper.get_axis_id(self.a_angular)

		# head mappings
		self.b_idx_neck_send    = key_mapper.get_button_id(self.b_neck_send)
		self.a_idx_neck_x       = key_mapper.get_axis_id(self.a_neck_x)
		self.a_idx_neck_y       = key_mapper.get_axis_id(self.a_neck_y)

		# emotion mapping
		self.b_idx_a        = key_mapper.get_button_id(self.b_a)
		self.b_idx_b        = key_mapper.get_button_id(self.b_b)
		self.b_idx_x        = key_mapper.get_button_id(self.b_x)
		self.b_idx_y        = key_mapper.get_button_id(self.b_y)

		# trigger mapping
		self.a_idx_trigger_l  = key_mapper.get_axis_id(self.a_trigger_l)
		self.a_idx_trigger_r  = key_mapper.get_axis_id(self.a_trigger_r)
		self.b_idx_trig_l	  = key_mapper.get_button_id(self.b_trig_l)
		self.b_idx_trig_r	  = key_mapper.get_button_id(self.b_trig_r)

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

			a_is_pressed    = msg.buttons[self.b_idx_a] == 1.0
			b_is_pressed    = msg.buttons[self.b_idx_b] == 1.0
			x_is_pressed    = msg.buttons[self.b_idx_x] == 1.0
			y_is_pressed    = msg.buttons[self.b_idx_y] == 1.0

			# left_triggered  = msg.axes[self.a_idx_trigger_l] < -0.9
			# right_triggered = msg.axes[self.a_idx_trigger_r] < -0.9

			left_triggered  = msg.buttons[self.b_idx_trig_l] == 1.0
			right_triggered = msg.buttons[self.b_idx_trig_r] == 1.0

			debugging       = left_triggered  and right_triggered
			l_arm_inst      = left_triggered  and not debugging
			r_arm_inst      = right_triggered and not debugging
			no_trigger      = not left_triggered and not right_triggered

			# TIMING PERFECTO MODIFICANDO tts.py linea time_to_sleep = letters/50.0
			if debugging:
				rospy.logwarn("Debugging commands")
				rospy.loginfo("Demonstration commands mode")
				if y_is_pressed:
					rospy.loginfo("Demonstration 1 executed...")
					self.tts.say("Hola")
					rospy.sleep(0.3)
					self.tts.wait_until_done()
					self.tts.say("Mi nombre es Bender")
					rospy.sleep(0.5)
					self.tts.wait_until_done()
					self.tts.say("Soy desarrollado en la Universidad de Chile")
					self.tts.wait_until_done()
					self.tts.say("Por el equipo Jom Breikers")
					rospy.sleep(0.2)
					self.tts.wait_until_done()
					self.tts.say("Me estan desarrollando desde el dos mil siete")
					self.tts.wait_until_done()
					self.tts.say("Soy un robot de servicio")
					rospy.sleep(0.2)
					self.tts.wait_until_done()
					self.tts.say("Mi principal funcion es servir e interactuar con las personas")
					self.tts.wait_until_done()
					self.face.set_emotion("happy1")
					self.tts.say("Para lograr mi cometido puedo emular emociones humanas")
					self.tts.wait_until_done()
					self.tts.say("Como la alegria")
					rospy.sleep(0.5)
					self.tts.wait_until_done()
					self.face.set_emotion("angry1")
					self.tts.say("Es mi estado basal y como me gusta estar")
					self.tts.wait_until_done()
					rospy.sleep(0.5)
					self.face.set_emotion("sad1")
					self.tts.say("Aunque si algo me molesta puedo hacerlo notar")
					self.tts.wait_until_done()
					self.face.set_emotion("fear")
					self.tts.say("Si me tratan mal me siento muy triste")
					rospy.sleep(0.5)
					self.tts.wait_until_done()
					self.tts.say("Y si noto que algo es peligroso para mi es muy facil asustarme")
					rospy.sleep(0.5)
					self.tts.wait_until_done()
					self.tts.say("Todo esto para ayudar a los humanos a comunicarse conmigo")
					self.tts.wait_until_done()
					self.face.set_emotion("happy1")

				if x_is_pressed:
					rospy.loginfo("Demonstration 2 executed...")
					self.tts.say("Existen diversas cosas que puedo hacer")
					rospy.sleep(0.2)
					self.tts.wait_until_done()
					self.tts.say("Dentro de mi movilidad se encuentra mi cabeza")
					self.tts.wait_until_done()
					self.neck.look_left()
					self.neck.wait_for_motion_done()
					self.neck.send_joint_goal(0.0,-abs(self.min_pitch_demo))
					self.neck.wait_for_motion_done()
					self.neck.look_right()
					self.neck.wait_for_motion_done()
					self.neck.look_at_ground()
					self.neck.wait_for_motion_done()
					self.neck.home()
					self.tts.say("Ademas de poder mover mis brazos")
					self.tts.wait_until_done()
					self.tts.say("Y con mis pinzas puedo hacer una manipulacion simple de objetos")
					self.tts.wait_until_done()
					self.tts.say("Finalmente me puedo mover con dos grados de libertad")
					self.tts.wait_until_done()
					self.tts.say("Una es la rotacion y la otra es el movimiento de adelante y atras")
					self.tts.wait_until_done()

				if a_is_pressed:
					rospy.loginfo("Demonstration 3 executed...")
					self.tts.say("El equipo humano que me desarrolla es voluntario")
					self.tts.wait_until_done()
					self.tts.say("Ellos, mis amigos, son estudiantes de la F C F M")
					rospy.sleep(0.3)
					self.tts.wait_until_done()
					self.tts.say("Entre sus carreras se encuentran Ingenieria civil electrica, mecanica, en computacion")
					self.tts.wait_until_done()
					self.neck.look_left()
					self.tts.say("Los integrantes del equipo van rotando en el tiempo")
					self.tts.wait_until_done()
					self.tts.say("Esto ya que ergesan e ingresan constantemente")
					self.tts.wait_until_done()
					self.neck.look_right()
					self.tts.say("Yo me actualizo y mejoro constantemente gracias a ellos")
					self.tts.wait_until_done()
					self.tts.say("Y ellos aprenden robotica gracias a mi")
					self.tts.wait_until_done()
					self.neck.home()
					self.tts.say("La plataforma en la que me desarrollan se llama ROS")
					rospy.sleep(0.2)
					self.tts.wait_until_done()
					self.tts.say("Es un entorno de trabajo que integra mis componentes en forma de nodos que se comunican entre ellos")
					self.tts.wait_until_done()

				if b_is_pressed:
					rospy.loginfo("Demonstration 4 executed...")

			if r_arm_inst:
				rospy.logwarn("Right arm commands mode")
				if y_is_pressed:
					rospy.loginfo("Opening gripper")
					self.r_gripper.open()
				if a_is_pressed:
					rospy.loginfo("Closing gripper")
					self.r_gripper.close()
				if x_is_pressed:
					rospy.loginfo("Moving right arm to home position")
					self.r_arm.send_joint_goal(self.r_arm_home_angles)
				if b_is_pressed:
					rospy.loginfo("Moving right arm to manipulation position")
					self.r_arm.send_joint_goal(self.r_arm_posUp_angles)

			if no_trigger:
				if neck_confirmed:
					if neck_to_left:
						yaw_neck = msg.axes[self.a_idx_neck_x] * abs(self.max_yaw_demo)
					if neck_to_right:
						yaw_neck = msg.axes[self.a_idx_neck_x] * abs(self.min_yaw_demo)
					if neck_to_up:
						pitch_neck = - msg.axes[self.a_idx_neck_y] * abs(self.min_pitch_demo)
					if neck_to_down:
						pitch_neck = - msg.axes[self.a_idx_neck_y] * abs(self.max_pitch_demo)
					self.neck.stop
					self.neck.send_joint_goal(yaw_neck,pitch_neck)

				if speak_tts1:
					rospy.loginfo("TTS1 pressed")
					self.tts.say(self.text_tts1)
				if speak_tts2:
					rospy.loginfo("TTS2 pressed")
					self.tts.say(self.text_tts2)
				if speak_tts3:
					rospy.loginfo("TTS3 pressed")
					self.tts.say(self.text_tts3)
				if speak_tts4:
					rospy.loginfo("TTS4 pressed")
					self.tts.say(self.text_tts4)

				if a_is_pressed:
					self.face.set_emotion("happy1")
					rospy.loginfo("Bender is happy :D")
				if b_is_pressed:
					self.face.set_emotion("angry1")
					rospy.loginfo("Bender is angry >:(")
				if x_is_pressed:
					self.face.set_emotion("sad1")
					rospy.loginfo("Bender is sad :c")
				if y_is_pressed:
					self.face.set_emotion("fear")
					rospy.loginfo("Bender is frightened D:")

			if msg.buttons[self.b_idx_priority]:
				rospy.logwarn_throttle(2, "Drive with care. Using the high priority joystick topic.")
				self.pub_priority.publish(cmd)
			else:
				self.pub.publish(cmd)

if __name__ == '__main__':
	rospy.init_node('joy_base')
	JoystickBase()
	rospy.spin()