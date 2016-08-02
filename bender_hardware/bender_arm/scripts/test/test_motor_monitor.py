#!/usr/bin/env python
import random
import rospy
from dynamixel_msgs.msg import MotorState, MotorStateList

def main():
    rospy.init_node('motor_monitor_test')
    pub = rospy.Publisher('motor_states', MotorStateList, queue_size = 2)
    motor_ids = [1,2,3]
    motor_list = MotorStateList()

    for m_id in motor_ids:
        m = MotorState()
        m.id = m_id
        motor_list.motor_states.append(m)
      
    rate = rospy.Rate(50)
    i=20
    while not rospy.is_shutdown():
        for state in motor_list.motor_states:
            state.load = random.random()
            i += 0.01
            state.temperature = min(i,80)
            state.error = random.randint(-20,20)
        pub.publish(motor_list)
        rate.sleep()

if __name__ == "__main__":
    main()