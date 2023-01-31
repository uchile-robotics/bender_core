#!/usr/bin/env python
import rospy
from bender_msgs.msg import Emotion

def _emotion(self):
        print("Emotions of Bender: Neutral, Happy, Sad, Angry and Surprise")
        emotion = input()
        output = Emotion()
        output.emotion = emotion
        self.head.publish(output)
        main()

class Emotion_Bender():
    def __init__(self):
        self.head = self.publisher = rospy.Publisher('/Emotion_Bender', Emotion , queue_size=10)
        _emotion(self)
    
    
def main():
    rospy.init_node('Emotion_Bender')
    Emotion_Bender()
    rospy.spin()

if __name__ == '__main__':
    main()
