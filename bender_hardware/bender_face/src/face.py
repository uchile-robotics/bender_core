#!/usr/bin/env python
# Matias Pavez Bahamondes
# 7/Nov/2014
# last: 22/mar/2015

import roslib
roslib.load_manifest('bender_face')

import rospy
import serial
from std_msgs.msg import String
from bender_msgs.msg import Emotion
from bender_srvs.srv import Float
from bender_srvs.srv import FloatRequest
from FaceSerialInterface import FaceSerialInterface
from HeadHTTPInterface import HeadHTTPInterface

# TODO: create service for: /bender_face/reached_neck_pose  
#  a'un no se puede, pk no se puede sensar la posicion actual 
class Face():
    
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
        face_type = rospy.get_param('~face_type', 'serial')

        # Hardware Interface (Handles (re)connections, writing and reading to the hardware)
        self.hardware_interface = None
        if face_type == 'serial':
            self.hardware_interface = FaceSerialInterface(self.msg_types)
            rospy.loginfo('Using Serial Face Interface')
        elif face_type == 'ethernet':
            self.hardware_interface = HeadHTTPInterface(self.msg_types)
            rospy.loginfo('Using Ethernet Face Interface')
        else:
            rospy.logerr("Unkown face type")
            exit('Bye. :p')

        # Connect to hardware
        rospy.loginfo("Connecting to face hardware . . . ")
        self.hardware_interface.connect()
        while not rospy.is_shutdown() and not self.hardware_interface.is_connected():
            rospy.logwarn("Connection Failed. Reconnecting . . . ")
            self.hardware_interface.connect()
            rospy.sleep(2)


        # - - - Preparation - - -
        
        # Usado para manejar movimiento de la boca
        self.last_state = 'Not Talking'
        self.set_emotion('speakoff')

        # Center head
        self.rotate_head(0)

        # Neutral emotion
        self.set_emotion('serious')
        
        # We are ready to process requests . . .
        # - - - Subscriptions - - -
        self.head_sub = rospy.Subscriber("head", Emotion, self.callback)
        self.mouth_sub = rospy.Subscriber("/bender/speech/synthesizer/move_mouth", String, self.start_move)
        self.status_sub = rospy.Subscriber("/bender/speech/synthesizer/status", String, self.check_talking_status)

        # all ok
        rospy.loginfo('Ready to work')


    def loop(self):
        self.hardware_interface.loop()

    def shutdown(self):
        
        rospy.loginfo('Turning face off: Setting to defaults')
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
                
    def start_move(self, info):
        ''' Inicializa el movimiento de la boca (lo logra antes que usando el callback '.../status')'''
        self.set_emotion("speakOn")

    def check_talking_status(self, info):
        ''' Utilizado para mantener la boca en movimiento y detenerla de ser necesario. '''

        status = info.data

        if self.last_state == status:
            #rospy.logwarn('no hago nada . . .')
            pass
            
        else:

            if status == "Not Talking":
                # orden cuando deja de hablar publicada por speech_sinthesizer/status
                self.set_emotion("speakOff")
                self.last_state = status
                rospy.loginfo('Me callo . . .')

            elif status == "Talking":
                self.set_emotion("speakOn")
                self.last_state = status
                rospy.loginfo('Empiezo a hablar . . .')

            else:
                #rospy.logwarn('lei basura :D')
                pass    

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
            
            rospy.loginfo("Setting face action: '" + emotion + "'")

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

    rospy.init_node('face')

    face = Face()

    # on python, spin() is only meant for blocking the main thread until ros shutdown.
    #rospy.spin()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        face.loop()
        r.sleep()

    rospy.loginfo('Closing connection . . .')
    face.shutdown()
    rospy.loginfo('Bye bye . . .')

if __name__ == '__main__':
    main()

