#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
__author__ = 'Matías Pavez'
__email__  = 'matias.pavez@ing.uchile.cl'

import rospy
from bender_core.robot import Robot
from bender_core.util.test_skill import TestSkill

if __name__ == "__main__":

    rospy.init_node("testbot_example")

    print "--------------------------------------------------------------------"
    rospy.loginfo(" building robot ... ")

    A = TestSkill("A")
    B = TestSkill("B")
    C = TestSkill("C")
    D = TestSkill("D")
    E = TestSkill("E")
    F = TestSkill("F")
    G = TestSkill("G")
    H = TestSkill("H")
    I = TestSkill("I")
    
    # A.register_dependency(C.get_type())
    # A.register_dependency(D.get_type())
    # A.register_dependency(E.get_type())
    # B.register_dependency(C.get_type())
    # C.register_dependency(F.get_type())
    # D.register_dependency(E.get_type())
    # D.register_dependency(G.get_type())
    A.register_dependency(H.get_type())
    B.register_dependency(C.get_type())
    B.register_dependency(D.get_type())
    E.register_dependency(F.get_type())
    E.register_dependency(G.get_type())
    E.register_dependency(H.get_type())
    G.register_dependency(I.get_type())
    H.register_dependency(I.get_type())


    robot = Robot("test bot")
    robot.set(A)
    robot.set(B)
    robot.set(C)
    robot.set(D)
    robot.set(E)
    robot.set(F)
    robot.set(G)
    robot.set(H)
    robot.set(I)

    rospy.loginfo(" ... ready")

    print "--------------------------------------------------------------------"
    print robot
    print "--------------------------------------------------------------------"

    # print "parents of A: " + str(robot.get_parents(A.get_type()))
    # print "parents of B: " + str(robot.get_parents(B.get_type()))
    # print "parents of C: " + str(robot.get_parents(C.get_type()))
    # print "parents of D: " + str(robot.get_parents(D.get_type()))
    # print "parents of E: " + str(robot.get_parents(E.get_type()))
    # print "parents of F: " + str(robot.get_parents(F.get_type()))
    # print "parents of G: " + str(robot.get_parents(G.get_type()))

    robot.start()
    robot._print_states()
    #D.start_return_value = False
    #D.pause_return_value = False
    #robot.start(A.get_type())
    #robot.start("asd")
    #robot._print_states()
    robot.shutdown(H.get_type(), whole_tree=True)
    #robot.pause(E.get_type())
    #robot.pause()
    #robot.shutdown()
    #robot.start()
    robot._print_states()

    print "--------------------------------------------------------------------"


