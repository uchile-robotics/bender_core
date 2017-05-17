#!/usr/bin/env python

import sys
import rospy
import scipy.io
import numpy as np

from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import ErrorCode

subscribe_once = None

class get_state():

    def __init__(self,arm_name):

        self.name = arm_name
        # subscribers
        global subscribe_once
        subscribe_once = rospy.Subscriber('/motor_states/'+self.name+'_port',MotorStateList,self.get_arm_state)
        rospy.Subscriber('/error_code/',ErrorCode,self.get_error_code)
        self.state = []
        self.data = np.zeros((1,50))
        # open file
        self.fo = open('/home/gonzalo/arm_joint_state.txt','a+')

    def get_arm_state(self,data):
        self.state = []
        for i in range(0,len(data.motor_states)):
            #print i
            self.state.append(data.motor_states[i].id)
            self.state.append(data.motor_states[i].goal)
            self.state.append(data.motor_states[i].position)
            self.state.append(data.motor_states[i].load)
            self.state.append(data.motor_states[i].voltage)
            self.state.append(data.motor_states[i].temperature)

        self.state.append(0)
        self.state.append(0)

    def get_error_code(self,data):

        self.state.append(data.id)
        self.state.append(data.error)

    def print_state(self):

        tmp = np.array([])
        tmp = self.state
        self.data = np.vstack((self.data,tmp))

        # write data
        print "Posicion guardada: ", str(self.state)
        self.fo.write(str(self.state) + '\n')

        #print "DATA: ", str(self.data)
        #scipy.io.savemat('/home/gonzalo/EL7014/data.mat',mdict={'arr':arr})_

    def close_file(self):
        # close file
        self.fo.close

    def save_mat(self):

        scipy.io.savemat('/home/gonzalo/EL7014/'+sys.argv[1]+'.mat',{'vect':self.data})

def main():
    asd = get_state('r_arm')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        asd.print_state()

    asd.save_mat()
    asd.close_file()


if __name__ == '__main__':

    rospy.init_node('print_arm_states')
    main()