#!/usr/bin/env python
import roslib
roslib.load_manifest('bender_gazebo')

from optparse import OptionParser
import rospy

parser = OptionParser()

if __name__ == '__main__':
    try:
        rospy.init_node('simple_log', anonymous=True)
        
        parser.add_option('-t', '--type', metavar='TYPE', default='info', choices=('info','warn','err'),
          help='type of message (info|warn|err) [default: %default]')
     
        (options, args) = parser.parse_args(rospy.myargv()[1:])
        
        if len(args) < 1:
            parser.error('specify a message')
          
        msg_type = options.type
        msg = args[0]
        r = rospy.Rate(1) # 1 Hz
        
        while not rospy.is_shutdown():
            if msg_type=='info':
                rospy.loginfo(msg)
            elif msg_type=='warn':
                rospy.logwarn(msg)
            else:
                rospy.logerr(msg)
            r.sleep()

    except rospy.ROSInterruptException:
        pass