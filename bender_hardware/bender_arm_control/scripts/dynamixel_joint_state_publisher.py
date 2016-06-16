#!/usr/bin/env python

"""
  dynamixel_joint_state_publisher.py
  
  Publish the dynamixel_controller joint states on the /joint_states topic
  
  Basado en software de Patrick Goebel. Pi Robot Project.

"""

import roslib
roslib.load_manifest('bender_arm_control')
import rospy

from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState as JointStateDynamixel

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

    def update(self, position, velocity, effort):
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)
        
        # Parametro Rate
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)

        # Parametro frame_id
        self.frame_id =  rospy.get_param('~frame_id', 'base_link')
        
        # Controllers
        self.controllers = rospy.get_param('~controllers', '')
        rospy.loginfo('controllers ' + str(self.controllers))

        self.joint_states = dict({})
        
        for controller in sorted(self.controllers):
            joint_name = rospy.get_param(controller + '/joint_name', '')
            #rospy.loginfo('Joint name ' + joint_name)
            self.joint_states[joint_name] = JointStateMessage(joint_name, 0.0, 0.0, 0.0)
                   
        # Subscribers
        [rospy.Subscriber(c + '/state', JointStateDynamixel, self.controller_state_handler) for c in self.controllers]
       
        # Publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        rospy.loginfo("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
         
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()
         
    def controller_state_handler(self, msg):
        # Actualizar joint state
        self.joint_states[msg.name].update(msg.current_pos, msg.velocity, msg.load)
       
    def publish_joint_states(self):
        # Construir mensaje
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        # Recolectar informacion
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
           
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        self.joint_states_pub.publish(msg)
    
if __name__ == '__main__':
    try:
        JointStatePublisher()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

