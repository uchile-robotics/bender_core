#!/usr/bin/env python
import rospy
import make_emotion as me
from bender_msgs.msg import Emotion, Mouth, Expression 



class Bender_Protocol():
    def __init__(self):
        self.sub = rospy.Subscriber('/Emotion_Bender', Emotion, self._callback)
        self.face = rospy.Publisher('/Bender_Protocol/Face', Expression , queue_size=10)
        self.mouth = rospy.Publisher('/Bender_Protocol/Mouth', Mouth , queue_size=10)

    def _callback(self, msg):
        emotion = msg.emotion
        ex = me.expression(emotion)
        # self.face.publish(ex[0])
        # self.mouth.publish(ex[1])

def main():

    rospy.init_node('Bender_Protocol')
    Bender_Protocol()
    rospy.spin()

if __name__ == '__main__':
    main()