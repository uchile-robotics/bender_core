#!/usr/bin/env python

import rospy
from uchile_turning_base.turning_base import TurningBase

def main():
    rospy.init_node('turning_base_test', anonymous=True)
    base = TurningBase()

    while not rospy.is_shutdown():
        base.turn(90,5000)
        rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
