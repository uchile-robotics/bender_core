#!/usr/bin/env python

import roslib; roslib.load_manifest('uchile_fun')
import rospy
import smach
import smach_ros

import pygame
import game as g
import settings as s

def getInstance():
	pygame.init()

	pygame.display.set_caption("Swervin Mervin")

	if s.FULLSCREEN:
	    w_flag = pygame.FULLSCREEN
	    # w_flag = pygame.RESIZABLE
	    pygame.mouse.set_visible(False)
	else:
	    w_flag = 0

	fps_clock = pygame.time.Clock()
	window    = pygame.display.set_mode(s.DIMENSIONS, w_flag)
	game      = g.Game(window, fps_clock)

	while True:
	   if game.waiting:
	       game.wait()
	   else:
	       game.play()


if __name__ == '__main__':
    # rospy.init_node('pyrace', anonymous=True)
    rospy.init_node('pyrace', anonymous=True)

    getInstance()