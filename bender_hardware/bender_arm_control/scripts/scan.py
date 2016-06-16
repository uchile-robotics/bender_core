#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import moveit_commander
import sys
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (JointTrajectoryAction, JointTrajectoryGoal,
  FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult)

class Head:
    def __init__(self):
        # Action server para movimiento de cabeza
        self.jta = actionlib.SimpleActionClient('/bender/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # Wait for action server
        rospy.loginfo('[%s] Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('[%s] Conected to joint trajectory action server')

    def move(self, angle):
        # Create a goal msg
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['head_pitch_joint']
        point = JointTrajectoryPoint()
        # Add angles
        point.positions = [angle]
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        # Send and wait
        rospy.loginfo('Sending goal and wait...')
        self.jta.send_goal_and_wait(goal)
        result = self.jta.get_result()
        # Check results
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo('Goal reached without error codes')
        elif result.error_code == FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            rospy.logwarn('Goal tolerance violated')
        else:
            rospy.logwarn('Error during trajectory execution (error_code = %s)' % result.error_code)

class Base:
    def __init__(self, rate = 30):
        # Topico para control de base
        self.pub = rospy.Publisher('/bender/nav/base/cmd_vel', Twist, queue_size=10)
        self.rate = rate # publish rate


    def move(self, linear_vel = 0, angular_vel = 0, time = 1):
        # Base msg
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        # Info
        rospy.loginfo('Move base with %s command' % [linear_vel, angular_vel, time])
        # Num de publicaciones
        n = time * self.rate
        r = rospy.Rate(self.rate)
        for i in xrange(n):
            self.pub.publish(msg)
            r.sleep()

    def stop(self):
        rospy.loginfo('Move base stop')
        self.move()

class BenderPlanning():
    def __init__(self):
        # Interface for robot
        self.robot = moveit_commander.RobotCommander()
        # Interface for world
        self.scene = moveit_commander.PlanningSceneInterface()
        # Interface for arms
        self.l_arm = BenderArmPlanning('l_arm', moveit_commander.MoveGroupCommander('l_arm'))
        self.r_arm = BenderArmPlanning('r_arm', moveit_commander.MoveGroupCommander('r_arm'))



class BenderArmPlanning():
    def __init__(self, arm_name, group_commander):
        self.arm_name = arm_name
        # Group
        self.group = group_commander
        self.joint_names = self.group.get_joints()
        rospy.loginfo('Interface for %s with %s joints' % (self.arm_name, self.joint_names))

    def moveJoints(self, angles):
        rospy.loginfo('Current joint values %s' % self.group.get_current_joint_values())
        rospy.loginfo('Request for %s with %s as goal joints values' % (self.arm_name, angles))
        self.group.set_joint_value_target(angles)
        rospy.loginfo('Planning for request...')
        self.group.plan()
        #plan = self.group.plan()
        #rospy.logwarn('Plan score: %s' % BenderArmPlanning.evalPlan(plan))
        rospy.loginfo('Ejecutando plan...')
        # try:
        #   self.group.go(wait=True)
        # except:
        #   e = sys.exc_info()[0]
        #   rospy.logerr('Exception on execution %s' % e)

    @staticmethod
    def evalPlan(plan):
        trajectory = plan.joint_trajectory.points
        # Suma de desplazamiento de joints en trayectoria
        score, partial, i = 0, 0, 0
        n = len(trajectory)
        while i < n-1:
            j, partial = 0, 0
            while j < len(trajectory[i].positions):
                partial += abs(trajectory[i+1].positions[j] - trajectory[i].positions[j])
                j+=1
            score += partial
            i+=1
        # Desplazamiento de joints
        # j, partial = 0, 0
        # while j < len(trajectory[n-1].positions):
        #   partial += abs(trajectory[0].positions[j] - trajectory[n-1].positions[j])
        #   j+=1
        # score = score / partial # Relativo
        return score


def main():
  
    rospy.init_node('bender_scan')
    # Init moveit commander
    #moveit_commander.roscpp_initialize(sys.argv)
    # Init
    head = Head()
    base = Base()
    #bender = BenderPlanning()
    # Scan
    head.move(1) # Turn down head
    base.move(linear_vel = -0.2, time = 3) # Move back
    base.move(angular_vel = -0.2, time = 3) # Scan left
    base.move(angular_vel = 0.2, time = 6) # Scan right
    base.move(angular_vel = -0.2, time = 3) # Come to center
    base.move(linear_vel = 0.2, time = 3) # Move front
    base.stop()
    ## Arm plan
    #bender.l_arm.moveJoints([0.22,0.31,0.0,1.57,0.55,0.0])
    #bender.l_arm.moveJoints([0.0,0.0,0.0,0.0,0.0,0.0])
    #bender.l_arm.moveJoints([-0.6,0.0,0.0,1.57,0.0,1.57])
    rospy.sleep(2)
    #moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
