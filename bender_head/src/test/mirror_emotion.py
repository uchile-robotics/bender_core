#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from bender_msgs.msg import Face

class Mirror_emotion():
    def __init__(self):
      self.emotion_sub = rospy.Subscriber("/emotion_detect", String, self._callback)
      self.publisher = rospy.Publisher('/face_controll', Face, queue_size=10)
      self.emotion = "Neutral"

    def _callback(self, msg):
      output = Face()

      self.emotion = msg.data
      
      output.ring_i  = [2,200]
      output.ring_d  = [2,200]

      if msg.data == "Happy":
        output.color_1 = [255,255,0]

      if msg.data == "Anger":
        output.color_1 = [255,0,0]

      if msg.data == "Sad":
        output.color_1 = [0,0,255]

      if msg.data == "Surprise":
        output.color_1 = [0,255,0]
      
      if msg.data == "Neutral":
        output.color_1 = [200,200,200]
      
      self.publisher.publish(output)

def color(self,msg):
  output = Face()
  buttons = msg.buttons
  a = buttons[0]
  b = buttons[1]
  x = buttons[2]
  y = buttons[3] 
  output.ring_i  = [2,200]
  output.ring_d  = [2,200]

  if a == 1:
    output.color_1 = [0,255,0]
  elif b == 1:
    output.color_1 = [255,0,0]
  elif x == 1:
    output.color_1 = [0,0,255]
  elif y == 1:
    output.color_1 = [255,255,0]
  else: 
    output.color_1 = [200,200,200]

  self.publisher.publish(output)

def joy(self,msg):

  axes = msg.axes
  buttons = msg.buttons
  output = Face()
  vertical = axes[1]
  horizontal= axes[0]
  stop = buttons[4]
  start = buttons[8]

  if stop== 1:
    output.pitch   = [500,1]
    output.yaw     = [750,1]       
  if start == 1:
    output.pitch   = [500,50]
    output.yaw     = [750,50]
  if vertical > 0.5:
    output.pitch = [420,50]
  if vertical < -0.5:
    output.pitch = [660,50]
  if horizontal > 0.5:
      output.yaw = [1010,50]
  if horizontal < -0.5:
    output.yaw = [480,50]

  self.publisher.publish(output)


def main():

    rospy.init_node('mirror')
    Mirror_emotion()
    rospy.spin()

if __name__ == '__main__':
    main()