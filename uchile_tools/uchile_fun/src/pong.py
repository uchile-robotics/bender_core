#!/usr/bin/env python

import roslib
roslib.load_manifest('uchile_fun')

import pygame
import sys
from pygame.locals import *

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# Number of frames per second
# Change this value to speed up or slow down your game
FPS = 200

#Global Variables to be used through our program

WINDOWWIDTH = 600
WINDOWHEIGHT = 750
LINETHICKNESS = 10
PADDLESIZE = 50
PADDLEOFFSET = 20

# Set up the colours
BLACK     = (0  ,0  ,0  )
WHITE     = (255,255,255)

RIGHT_LEFT=0

#Draws the arena the game will be played in. 
def drawArena():
    DISPLAYSURF.fill((0,0,0))
    #Draw outline of arena
    pygame.draw.rect(DISPLAYSURF, WHITE, ((0,0),(WINDOWWIDTH,WINDOWHEIGHT)), LINETHICKNESS*2)
    #Draw centre line
    pygame.draw.line(DISPLAYSURF, WHITE, (0,(WINDOWHEIGHT/2)),(WINDOWWIDTH,(WINDOWHEIGHT/2)), (LINETHICKNESS/4))


#Draws the paddle
def drawPaddle(paddle):
    #Stops paddle moving too low
    if paddle.right > WINDOWWIDTH - LINETHICKNESS:
        paddle.right = WINDOWWIDTH - LINETHICKNESS
    #Stops paddle moving too high
    elif paddle.left < LINETHICKNESS:
        paddle.left = LINETHICKNESS
    #Draws paddle
    pygame.draw.rect(DISPLAYSURF, WHITE, paddle)


#draws the ball
def drawBall(ball):
    pygame.draw.rect(DISPLAYSURF, WHITE, ball)

#moves the ball returns new position
def moveBall(ball, ballDirX, ballDirY):
    ball.x += ballDirX 
    ball.y += ballDirY 
    return ball

#Checks for a collision with a wall, and 'bounces' ball off it.
#Returns new direction
def checkEdgeCollision(ball, ballDirX, ballDirY):
    if ball.top == (LINETHICKNESS) or ball.bottom == (WINDOWHEIGHT - LINETHICKNESS):
        ballDirY = ballDirY * -1
    if ball.left == (LINETHICKNESS) or ball.right == (WINDOWWIDTH - LINETHICKNESS):
        ballDirX = ballDirX * -1
    return ballDirX, ballDirY

#Checks is the ball has hit a paddle, and 'bounces' ball off it.     
def checkHitBall(ball, paddle1, paddle2, ballDirY):
    if ballDirY == 1 and paddle1.top == ball.bottom and paddle1.left - LINETHICKNESS < ball.left and paddle1.right + LINETHICKNESS> ball.right:
        return -1
    elif ballDirY == -1 and paddle2.bottom == ball.top and paddle2.left - LINETHICKNESS< ball.left and paddle2.right  + LINETHICKNESS> ball.right:
        return -1
    else:
        return 1

#Checks to see if a point has been scored returns new score
def checkPointScored(paddle1, ball, score, ballDirY):
    #reset points if left wall is hit
    if ball.bottom == WINDOWHEIGHT - LINETHICKNESS: 
        return 0
    #1 point for hitting the ball
    elif ballDirY == 1 and paddle1.top == ball.bottom and paddle1.left - LINETHICKNESS< ball.left and paddle1.right + LINETHICKNESS> ball.right:
        score += 1
        return score
    #5 points for beating the other paddle
    elif ball.top == LINETHICKNESS:
        score += 5
        return score
    #if no points scored, return score unchanged
    else:
        return score

#Artificial Intelligence of computer player 
def artificialIntelligence(ball, ballDirY, paddle2):
    #If ball is moving away from paddle, center bat
    if ballDirY == 1:
        if paddle2.centerx < (WINDOWWIDTH/2):
            paddle2.x += 1
        elif paddle2.centery > (WINDOWWIDTH/2):
            paddle2.x -= 1
    #if ball moving towards bat, track its movement. 
    elif ballDirY == -1:
        if paddle2.centerx < ball.centerx:
            paddle2.x += 1
        else:
            paddle2.x -=1
    return paddle2

#Displays the current score on the screen
def displayScore(score):
    resultSurf = BASICFONT.render('Score = %s' %(score), True, WHITE)
    resultRect = resultSurf.get_rect()
    resultRect.topleft = (WINDOWWIDTH - 150, 25)
    DISPLAYSURF.blit(resultSurf, resultRect)

#Main function
def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/bender/joy/joy0", Joy, callback)

    global RIGHT_LEFT


    pygame.init()
    global DISPLAYSURF
    ##Font information
    global BASICFONT, BASICFONTSIZE
    BASICFONTSIZE = 20
    BASICFONT = pygame.font.Font('freesansbold.ttf', BASICFONTSIZE)

    FPSCLOCK = pygame.time.Clock()
    DISPLAYSURF = pygame.display.set_mode((WINDOWWIDTH,WINDOWHEIGHT)) 
    pygame.display.set_caption('Pong')

    #Initiate variable and set starting positions
    #any future changes made within rectangles
    ballX = WINDOWWIDTH/2 - LINETHICKNESS/2
    ballY = WINDOWHEIGHT/2 - LINETHICKNESS/2
    playerOnePosition = (WINDOWWIDTH - PADDLESIZE) /2
    playerTwoPosition = (WINDOWWIDTH - PADDLESIZE) /2
    score = 0

    #Keeps track of ball direction
    ballDirX = -1 ## -1 = left 1 = right
    ballDirY = -1 ## -1 = up 1 = down

    #Creates Rectangles for ball and paddles.
    paddle1 = pygame.Rect(playerOnePosition,WINDOWHEIGHT - PADDLEOFFSET - LINETHICKNESS, PADDLESIZE,LINETHICKNESS)
    paddle2 = pygame.Rect(playerTwoPosition, PADDLEOFFSET, PADDLESIZE/2,LINETHICKNESS)
    ball = pygame.Rect(ballX, ballY, LINETHICKNESS, LINETHICKNESS)

    #Draws the starting position of the Arena
    drawArena()
    drawPaddle(paddle1)
    drawPaddle(paddle2)
    drawBall(ball)

    pygame.mouse.set_visible(0) # make cursor invisible

    while 1: #main game loop
        for event in pygame.event.get():
            #Keydown
            # if event.type == pygame.QUIT:
            #     pygame.quit()
            #     quit()
                
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit(0)
        #for event in pygame.event.get():
          #  if event.type == QUIT:
           #     score = 3
            #    pygame.quit()
             #   sys.exit()
            # mouse movement commands
            # elif event.type == MOUSEMOTION:
            #     # print paddle1.x,  event.pos
            #     # print data.axes[0]
            #     mousex, mousey = event.pos
            #     paddle1.x = mousex
        if score == 3:
            pygame.quit()
            sys.exit()

        # if JOYMOTION == 1:

        if RIGHT_LEFT>0 or RIGHT_LEFT==1:
            paddle1.x = paddle1.x-2
        if RIGHT_LEFT<0 or RIGHT_LEFT==-1:
            paddle1.x = paddle1.x+2


        drawArena()
        drawPaddle(paddle1)
        drawPaddle(paddle2)
        drawBall(ball)

        ball = moveBall(ball, ballDirX, ballDirY)
        ballDirX, ballDirY = checkEdgeCollision(ball, ballDirX, ballDirY)
        score = checkPointScored(paddle1, ball, score, ballDirY)
        ballDirY = ballDirY * checkHitBall(ball, paddle1, paddle2, ballDirY)
        paddle2 = artificialIntelligence (ball, ballDirY, paddle2)

        displayScore(score)

        pygame.display.update()
        FPSCLOCK.tick(FPS)
        #rospy.spin()        

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", Joy)

    global RIGHT_LEFT

    RIGHT_LEFT=data.axes[0]


    if data.buttons[2]==1:
        print "boton  azul"
    if data.buttons[3]==1:
        print "boton  amarillo"
    if data.buttons[1]==1:
        print "boton  rojo"
    if data.buttons[0]==1:
        print "boton  verde"
    if data.axes[0]>0:
        print "mueve a la izquierda",data.axes[0]  
    if data.axes[0]<0:
        print "mueve a la derecha", data.axes[0]



if __name__=='__main__':
    main()
