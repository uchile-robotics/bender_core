#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (
    JointTrajectoryAction,
    JointTrajectoryGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult
)

class Arm:
    def __init__(self, arm_name):
        # arm_name should be r_arm or l_arm
        self.arm_name = arm_name
        # Load joint names from parameter server (loaded in lauch file)
        joint_names = rospy.get_param('/bender/arm/joint_names')
        # Action server
        #rospy.loginfo('/bender/'+self.arm_name+'_controller/follow_joint_trajectory')
        self.jta = actionlib.SimpleActionClient('/bender/'+self.arm_name 
                        + '_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        # Joint names
        side = self.arm_name[0] # either 'l' or 'r'
        self.joint_names = [side + '_' + joint for joint in joint_names] # side_joint
        rospy.loginfo('[%s] Client for [%s]' % (self.arm_name, self.joint_names))
        # Wait for action server
        rospy.loginfo('[%s] Waiting for joint trajectory action' % self.arm_name)
        self.jta.wait_for_server()
        rospy.loginfo('[%s] Conected to joint trajectory action server' % self.arm_name)

    def move_joint(self, angles):
        # Create a goal msg
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        # Add angles
        point.positions = angles
        point.time_from_start = rospy.Duration(2)
        goal.trajectory.points.append(point)
        # Send and wait
        rospy.loginfo('[%s] Sending goal and wait...' % self.arm_name)
        self.jta.send_goal_and_wait(goal)
        result = self.jta.get_result()
        # Check results
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo('[%s] Goal reached without error codes' % self.arm_name)
        elif result.error_code == FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            rospy.logwarn('[%s] Goal tolerance violated' % self.arm_name)
        else:
            rospy.logwarn(
                '[%s] Error during trajectory execution (error_code = %s)' % (self.arm_name, result.error_code))
      
def main():
    # Create arms action clients
    l_arm = Arm('l_arm')
    r_arm = Arm('r_arm')
    # Send joint action
    #l_arm.move_joint([0.45, 0.0, 0.0, 0.0, 0.0, 0.0])
    l_arm.move_joint([0.45, 0.3, 1.0, 1.0, 0.6, 0.4])
    r_arm.move_joint([0.95, 0.3, 1.2, 1.0, 0.6, 0.4])
    rospy.sleep(5)
    #l_arm.move_joint([-0.45, 0.0, 0.0, 0.0, 0.0, 0.0])
    l_arm.move_joint([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    r_arm.move_joint([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()