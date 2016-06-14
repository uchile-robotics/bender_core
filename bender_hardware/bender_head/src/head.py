#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Matias Pavez Bahamondes

import roslib
roslib.load_manifest('bender_head')

import rospy
import serial
from std_msgs.msg import Bool
from bender_msgs.msg import Emotion
from bender_srvs.srv import Float
from bender_srvs.srv import FloatRequest


# TODO: create service for: /bender/head/reached_neck_pose  
#  a'un no se puede, pk no se puede sensar la posicion actual 
class Head():
    
    def __init__(self):

        # different message types
        self.msg_types = ['mouth', 'yaw', 'emotion']

        # valid emotions        
        self.emotion_list = [
            'serious'  , 'happy1'  , 'happy2', 'happy3'  , 'sad1'   , 'sad2'   , 'sad3',
            'angry1'   , 'angry2'  , 'angry3', 'surprise', 'ashamed', 'standby', 'eyebrow',
            'greetings', '1313'    , 'ear'   , 'yes'     , 'no'     , 'lost'   , 'relaxed',
            'normal'   , 'agitated', 'blink' , 'flirt'   , 'speakon', 'speakoff'
        ]

        # Parameter Server
        interface = rospy.get_param('~interface', 'serial')

        # Hardware Interface (Handles (re)connections, writing and reading to the hardware)
        self.hardware_interface = None
        if interface == 'serial':
            from bender_head import HeadSerialInterface as HSI
            self.hardware_interface = HSI.HeadSerialInterface(self.msg_types)
            rospy.loginfo('Using Serial Head Interface')

        elif interface == 'ethernet':
            from bender_head import HeadHTTPInterface as HHI
            self.hardware_interface = HHI.HeadHTTPInterface(self.msg_types)
            rospy.loginfo('Using Ethernet Head Interface')

        elif interface == 'mock':
            from bender_head import HeadMockInterface as HMI
            self.hardware_interface = HMI.HeadMockInterface(self.msg_types)
            rospy.logwarn("Using Mock Head Interface")

        else:
            rospy.logerr("Unkown head port type")
            exit('Bye. :p')


        # - - - Preparation - - -
        
        # Usado para manejar movimiento de la boca
        self.was_talking = False
        self.set_emotion('speakoff')

        # Center head
        self.rotate_head(0)

        # Neutral emotion
        self.set_emotion('serious')
        
        # We are ready to process requests . . .
        # - - - Subscriptions - - -
        self.head_sub = rospy.Subscriber("~cmd", Emotion, self.callback)
        self.mouth_sub = rospy.Subscriber("~move_mouth", Bool, self.move_mouth_callback)

        # all ok
        rospy.loginfo('Ready to work')


    def loop(self):
        self.hardware_interface.loop()

    def shutdown(self):
        
        rospy.loginfo('Turning head off: Setting to defaults')
        self.rotate_head(0)
        self.set_emotion('serious')
        self.hardware_interface.loop()
        self.hardware_interface.close()

    def write(self, key, value):
        self.hardware_interface.write(key, value)
    
    def callback(self, info):
        
        emotion = info.Action.lower()
        order = info.Order.lower()
        angle = info.X
        
        if order=="changeface" or order=="emotion":
            self.set_emotion(emotion)
        elif order=="movex" or order=="yaw":
            self.rotate_head(angle)
        else:
            rospy.logwarn("Unkown order: '" + order + "' ... Please use 'changeFace/emotion' or 'MoveX/yaw' orders.")
            

    def move_mouth_callback(self, msg):
        """
        callback para tópico "move_mouth". Inicia o detiene movimiento de la boca
        """

        is_talking = msg.data

        # do nothing
        if self.was_talking and is_talking:
            return
            
        if is_talking:
            # started talking
            self.set_emotion("speakOn")
            self.was_talking = True
            rospy.loginfo('Empiezo a hablar . . .')

        else:
            # stopped talking
            self.set_emotion("speakOff")
            self.was_talking = False
            rospy.loginfo('Me callo . . .')


    def set_emotion(self, emotion):

        emotion = emotion.lower()
        if emotion in self.emotion_list:

            # speakOn/Off patch: send as special emotion type
            if emotion == 'speakoff':
                self.write('mouth','off')
            elif emotion == 'speakon':
                self.write('mouth','on')
            else:
                self.write('emotion',emotion)
            
            rospy.loginfo("Setting head action: '" + emotion + "'")

        else:
            rospy.logwarn("For order 'changeFace', unknown action: '" + emotion 
                + "' ... Please use one of the following:\n" + str(self.emotion_list))

    def rotate_head(self, angle):
        """
        Rotates head to the desired angle [degrees]
        
        Use the right hand rule. (turn left > 0 > turn right)
        """

        # software constraint
        # TODO: get from PIC
        if angle > 55 or angle < -55:
            rospy.logwarn("angle %.2f out of range",angle)
            return

        rospy.loginfo("Setting head direction to: " + str(angle) + " [degrees]'")
        self.write('yaw', str(angle))
        self.update_tf_tree(angle)

    def update_tf_tree(self,angle):

        # TODO
        #

        # update_client = rospy.ServiceProxy('/bender/tf/neck_tf/update', Float)
        # req = FloatRequest()
        # req.x1 = angle

        # try:    
        #     update_client(req)

        # except:
        #     # No nos interesa esperar bloquear el 
        #     # nodo, si es que no se usar'a junto a las tf's
        #     pass
        pass


def main():

    rospy.init_node('head')

    head = Head()

    # on python, spin() is only meant for blocking the main thread until ros shutdown.
    #rospy.spin()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        head.loop()
        r.sleep()

    rospy.loginfo('Closing connection . . .')
    head.shutdown()
    rospy.loginfo('Bye bye . . .')

if __name__ == '__main__':
    main()

