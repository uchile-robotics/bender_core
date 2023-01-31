#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Joy
from bender_msgs.msg import Face

class Face_joy():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/joy', Joy, self._callback)
        self.publisher = rospy.Publisher('/face_controll', Face, queue_size=10)

    def _callback(self, msg):
      color(self,msg)
      joy(self,msg)
        

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

    rospy.init_node('face_joy')
    Face_joy()
    rospy.spin()

if __name__ == '__main__':
    main()
