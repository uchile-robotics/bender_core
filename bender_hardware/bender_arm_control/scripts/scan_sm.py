#!/usr/bin/env python
import roslib
roslib.load_manifest('bender_arm_control')
import rospy
import smach
import smach_ros

import actionlib
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (JointTrajectoryAction, JointTrajectoryGoal,
  FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult)

class Head(smach.State):
    def __init__(self):
        # SM
        smach.State.__init__(self,
            outcomes=['succeeded','aborted'],
            input_keys=['angle'])

        # Action server para movimiento de cabeza
        self.jta = actionlib.SimpleActionClient('/bender/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # Wait for action server
        rospy.loginfo('[%s] Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('[%s] Conected to joint trajectory action server')

    def execute(self, userdata):
        # Get result
        result = self.move(userdata.angle)
        # Check results
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo('Goal reached without error codes')
            return 'succeeded'
        elif result.error_code == FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            rospy.logwarn('Goal tolerance violated')
            return 'succeeded'
        else:
            rospy.logwarn('Error during trajectory execution (error_code = %s)' % result.error_code)
            return 'aborted'

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
        return self.jta.get_result()
      

class Base:
    def __init__(self, rate = 30):
        # Topico para control de base
        self.pub = rospy.Publisher('/bender/cmd_vel', Twist, queue_size=10)
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

def maina():
    rospy.init_node('bender_scan')
    head = Head()
    base = Base()
    head.move(1) # Turn down head
    base.move(linear_vel = -0.2, time = 3) # Move back
    base.move(angular_vel = -0.2, time = 3) # Scan left
    base.move(angular_vel = 0.2, time = 6) # Scan right
    base.move(angular_vel = -0.2, time = 3) # Come to center
    base.move(linear_vel = 0.2, time = 3) # Move front
    base.stop()



# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        rospy.sleep(3)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(3)
        return 'outcome2'
      
# main
def main():
    rospy.init_node('scan_sm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                     transitions={'outcome1':'BAR',
                          'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                     transitions={'outcome2':'FOO'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('bender_sm_server', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

