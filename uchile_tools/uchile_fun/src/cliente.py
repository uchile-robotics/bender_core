#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

X=0
Y=0
A=0
B=0
RIGHT_LEFT=0
bandera = 0

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", Joy)
    global X, Y, bandera 
    X =data.buttons[2]
    Y =data.buttons[3]
    #A =data.buttons[0]
    #B =data.buttons[1]
    #RIGHT_LEFT =data.axes[0]
    bandera =1


    # if data.buttons[2]==1:
    #   print "boton  azul"
    # if data.buttons[3]==1:
    #   print "boton  amarillo"
    # if data.buttons[1]==1:
    #   print "boton  rojo"
    # if data.buttons[0]==1:
    #   print "boton  verde"
    # if data.axes[0]>0:
    #   print "mueve a la izquierda"
    # if data.axes[0]<0:
    #   print "mueve a la derecha"

def listen_joy():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/bender/joy/joy0", Joy, callback)

    while 1:
        #print X
        global X, Y, bandera

        if bandera==1:
            if X==1:
                print "boton  azul"
            if Y==1:
                print "boton  amarillo"
            if B==1:
                print "boton  rojo"
            if A==1:
                print "boton  verde"
            if RIGHT_LEFT>0:
                print "mueve a la izquierda"
            if RIGHT_LEFT<0:
                print "mueve a la derecha"
            bandera =0  

    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listen_joy()
        