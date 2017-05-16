#==================================================================
#					BENDER SPACE
#
#		Game developed by Lorena Sanchez & Dario Palma
#				Based on MVC design pattern
#==================================================================

#==================================================================
#                     IMPORTS
#==================================================================

#>> ROS & BENDER
import roslib; roslib.load_manifest('uchile_fun')
import rospy
import smach
import smach_ros
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from bender_macros.head import FaceOrder
from bender_macros.speech import Talk

#>> PYGAME
import sys
import time
import random
import pygame
from pygame.constants import FULLSCREEN, KEYDOWN, K_LEFT, K_RIGHT, K_SPACE, K_ESCAPE, K_RETURN

import rospkg
rospack = rospkg.RosPack()
#==================================================================
#                      CONSTANTS
#==================================================================

# Screen Dimensions
WIDTH = 1200
HEIGHT = 700

# General Parameters
NAOLIMIT = 500
MOTHERSHIPHEALTH = 15
MAX_ROCKETS = 2
MAX_BALLS = 3
PLAYER_LIVES = 3

# Colors
WHITE = (255,255,255)
BLUE = (0,0,190)
GRAY = (150,150,150)
RED = (180,0,0)
BROWN = (50,50,0)
BLACK = (0,0,0)
GREEN = (0,190,0)
YELLOW = (230,220,120)

RIGHT_LEFT=0
B =0
START =0
EXIT =0
# File Paths
PATH = rospack.get_path('uchile_fun') + "/src/uchile_fun/BenderVaders/"

MOTHERSHIP_A = PATH+"./Assets/Images/Mothership.png"
MOTHERSHIP_B = PATH+"./Assets/Images/Mothershipb.png"
PLAYER_IMAGE = PATH+"./Assets/Images/Player.png"
ALIEN1A = PATH+"./Assets/Images/Alien1a.png"; ALIEN1B = PATH+"./Assets/Images/Alien1b.png"
ALIEN2A = PATH+"./Assets/Images/Alien2a.png"; ALIEN2B = PATH+"./Assets/Images/Alien2b.png"
ALIEN3A = PATH+"./Assets/Images/Alien3a.png"; ALIEN3B = PATH+"./Assets/Images/Alien3b.png"
PLAYER_ROCKET = PATH+"./Assets/Images/Missile_Player.png"
NAO_BALL = PATH+"./Assets/Images/Missile_Alien.png"
WIN_IMAGE = PATH+"./Assets/Images/Win.png"
LOGO = PATH+"./Assets/Images/Logo.png"
CONTROLS_IMAGE = PATH+"./Assets/Images/Controls.png"

OUCH_SOUND = PATH+"./Assets/Sounds/ouch2.wav"
LASER_SOUND = PATH+"./Assets/Sounds/laser6.wav"

WIN_MUSIC = PATH+"./Assets/Sounds/levelWin.mp3"
BOSS_MUSIC = PATH+"./Assets/Sounds/final_battle.mp3"
GAMEOVER_MUSIC = PATH+"./Assets/Sounds/gameOver.mp3"
GAME_MUSIC = PATH+"./Assets/Sounds/game-game_0.mp3"
MENU_MUSIC = PATH+"./Assets/Sounds/menu.mp3"

#==================================================================
#                      PROJECTILE
#==================================================================

class Projectile(pygame.sprite.Sprite):
	def __init__(self, img, x, y,speed=4, m=0):
		pygame.sprite.Sprite.__init__(self)
		
		self.image = pygame.image.load(img)
		self.rect = self.image.get_rect()
		self.rect.x = x
		self.rect.y = y
		self.speed = speed
		self.m = m
		
	def update(self):
		# Moves the projectile
		pygame.sprite.Sprite.update(self, None)
		self.rect.y += self.speed
		# If the projectile belongs to the boss the m won't be 0 and it will move on the X axis
		if self.m!=0:
			self.rect.x += self.speed*self.m 
		
		# If the projectile leaves the area kill()
		if self.rect.y < 0:
			self.kill()
			
		elif self.rect.y > HEIGHT-100:
			self.kill()
		
		if self.rect.x <0:
			self.kill()
		elif self.rect.x > WIDTH:
			self.kill()

#==================================================================
#                      MOTHERSHIP
#==================================================================
class MotherShip(pygame.sprite.Sprite):
	def __init__(self, x, y):
		pygame.sprite.Sprite.__init__(self)
		
		self.im1 = pygame.image.load(MOTHERSHIP_A)
		self.im2 = pygame.image.load(MOTHERSHIP_B)
		self.image = self.im1
		self.rect = self.image.get_rect()
		self.imAnimate = 0
		
		self.rect.x = x-self.rect.w/2
		self.rect.y = y
		
		self.health = MOTHERSHIPHEALTH
		
	def moveH(self, speed, verticalPos):
		speedChange = -1
		if WIDTH-self.rect.w-50 < self.rect.x:
			self.rect.x -=2
		elif self.rect.x < 50:
			self.rect.x +=2
		else:
			self.rect.x += speed
			speedChange*=-1
		return speedChange
			
	def animate(self):
		if self.imAnimate:
			self.image = self.im2
			self.imAnimate = 0
		else:
			self.image = self.im1
			self.imAnimate = 1
	
	def diminishHealth(self):
		self.health-=1
		
	def getHealth(self):
		return self.health
	
	def getPos(self):
		return self.rect

#==================================================================
#                         NAO
#==================================================================
class Nao(pygame.sprite.Sprite):
	def __init__(self,x,y, im_1, im_2):
		pygame.sprite.Sprite.__init__(self)
		
		self.im1 = pygame.image.load(im_1)
		self.im2 = pygame.image.load(im_2)
		
		self.image = self.im1
		self.rect = self.image.get_rect()
		self.imAnimate = 0
		
		self.rect.x = x
		self.rect.y = y
		
	def moveH(self, Hspeed, vPos):
		self.rect.x += Hspeed
		self.rect.y += vPos
		return self.rect.y>(HEIGHT-self.rect.h)
		
	def animate(self):
		if self.imAnimate:
			self.image = self.im2
			self.imAnimate = 0
		else:
			self.image = self.im1
			self.imAnimate = 1
	
	def getPos(self):
		return self.rect

#==================================================================
#                    	    PLAYER
#==================================================================
class Player(pygame.sprite.Sprite):
	def __init__(self, x, y):
		pygame.sprite.Sprite.__init__(self)
		
		self.image = pygame.image.load(PLAYER_IMAGE)
		self.rect = self.image.get_rect()
		
		self.rect.x = x
		self.rect.y = y
		
		self.lives = PLAYER_LIVES
		self.diedBy = None
		
	def moveH(self, speed):
		if WIDTH-self.rect.w-50 < self.rect.x:
			self.rect.x -=2
		elif self.rect.x < 50:
			self.rect.x +=2
		else:
			self.rect.x += speed
		
	def looseLive(self, loose=1):
		self.lives -=loose
		
	def getPos(self):
		return self.rect
	
	def getLives(self):
		return self.lives

	def setDeath(self, cause):
		self.diedBy = cause

#==================================================================
#                  		      MODEL
#==================================================================

class Model: 
	def __init__(self):
		self.player = Player(WIDTH/2,HEIGHT-100)
		self.rocketSound = pygame.mixer.Sound(LASER_SOUND)
		self.hitSound = pygame.mixer.Sound(OUCH_SOUND)
		self.naoList = []
		self.naoSpeed = 1
		self.moved = 0
		self.maxPlayerRockets = MAX_ROCKETS
		self.maxNaoBalls = MAX_BALLS
		self.naosOnMap = 0
		self.bossTime = False
		self.boss = None
		
		self.naos = pygame.sprite.Group()
		self.balls = pygame.sprite.Group()
		self.players = pygame.sprite.Group()
		self.rockets = pygame.sprite.Group()
		
		self.players.add(self.player)
		
		self.generateNaos()
		
	def generateNaos(self): # Generates the nao enemies at the first frame
		v_quantity = WIDTH/100 - 5 
		h_quantity = 4
		
		aliens = [ALIEN1A,ALIEN1B,ALIEN2A,ALIEN2B,ALIEN2A,ALIEN2B,ALIEN3A,ALIEN3B]
		
		v_index = h_quantity*2-1
		for i in range(0,v_index,2):
			for j in range(0,v_quantity,1):
				nao = Nao(100*j,10+i*40, aliens[i], aliens[i+1])
				self.naos.add(nao)
				
		self.naosOnMap = v_quantity*h_quantity
		
	def draw(self,screen):
		self.players.draw(screen)
		self.naos.draw(screen)
		self.rockets.draw(screen)
		self.balls.draw(screen)
	
	def move(self): # Moves the model elements
		self.moved +=1
		
		moveVertical = 0
		naoList = self.naos.sprites()
		
		if self.moved >NAOLIMIT and not self.bossTime:
			self.naoSpeed*=-1
			self.moved = 0
			moveVertical = 100
			
		for i in range(self.naosOnMap):
			currentNao = naoList[i] 
			naoReachedFloor = currentNao.moveH(self.naoSpeed,moveVertical)
			if naoReachedFloor and not self.bossTime:
				self.player.looseLive(3)
			# Player died when naos got to the floor
				self.player.setDeath(2)
			if self.bossTime:
				self.naoSpeed*=naoReachedFloor
					
		self.rockets.update()
		self.balls.update()
		
		# Check Collitions
		if not self.bossTime:
			self.checkCollitions()
		else:
			self.checkBossLife()
			
		self.checkPlayerCollitions()
		
	def animateNaos(self):
		naoList = self.naos.sprites()
		for i in range(self.naosOnMap):
			naoList[i].animate()
			
	def checkCollitions(self):
		# Checks if the player's Rockets hit a Nao
		rockets = self.rockets.sprites()
		rocketsQuantity = len(rockets)
		for i in range(0,rocketsQuantity,1):
			collides = pygame.sprite.spritecollide(rockets[i], self.naos, True)
			if len(collides)!=0:
				rockets[i].kill()
				#======================BENDER EMOTION==================
				FaceOrder.ChangeFace("happy2")

				self.naosOnMap -= 1
				if self.naosOnMap==0 and not self.bossTime:
					# Here the game conditions change
					self.boss = MotherShip(WIDTH/2, 10)
					self.naos.add(self.boss)
					self.naosOnMap=1
					self.bossTime = True
					self.maxNaoBalls +=3
					pygame.mixer.music.stop()
					pygame.mixer.music.load(BOSS_MUSIC)
					pygame.mixer.music.play(-1)
					
	def checkPlayerCollitions(self):
		# Checks if the Nao's Balls hit the player
		playerCollides = pygame.sprite.spritecollide(self.player, self.balls, True)
		if len(playerCollides) !=0:
			self.player.looseLive()
			# Set last hit to remember what killed bender
			self.player.setDeath(0)

			pygame.mixer.Sound.play(self.hitSound)
			#======================BENDER EMOTION==================
			FaceOrder.ChangeFace("surprise")

		playerCollidesWithNao = pygame.sprite.spritecollide(self.player, self.naos, False)
		if len(playerCollidesWithNao) !=0:
			self.player.looseLive(3)
			# Set last hit to remember what killed bender
			self.player.setDeath(1)

			pygame.mixer.Sound.play(self.hitSound)
			#======================BENDER EMOTION==================
			FaceOrder.ChangeFace("surprise")
			
	def checkBossLife(self):
		rockets = self.rockets.sprites()
		rocketsQuantity = len(rockets)
		for i in range(0,rocketsQuantity,1):
			collides = pygame.sprite.spritecollideany(rockets[i], self.naos)
			if collides!=None:
				rockets[i].kill() 
				self.boss.diminishHealth()
				if self.boss.getHealth()==0:
					self.naos.empty()
					self.naosOnMap = 0
		
	def movePlayer(self,Hspeed):
		self.player.moveH(Hspeed)
	
	def playerRocket(self):
		# Limits the players rockets
		if len(self.rockets.sprites()) < self.maxPlayerRockets:
			(x,y,w,h)= self.player.getPos()
			rocket = Projectile(PLAYER_ROCKET, x+w/2, y, -4)
			self.rockets.add(rocket)
			pygame.mixer.Sound.play(self.rocketSound)
	
	def spawnNaoBalls(self):
		naos = self.naos.sprites()
		if len(self.balls.sprites()) < self.maxNaoBalls and self.naosOnMap!=0:
			if self.bossTime:
				(x,y,w,h) = naos[0].getPos()
				pos = random.randint(0,5)
				xpos = w/5.0*pos
				(px, py,pw,ph) = self.player.getPos()
				# The projectile will aim to the player
				dy = (y+h-py) 
				dx = (x+xpos-px)
				ball = Projectile(NAO_BALL,x+xpos,y+h,6,dx/dy)
				self.balls.add(ball)
			else:
				index = random.randint(0,self.naosOnMap-1)
				(x,y,w,h) = naos[index].getPos()
				ball = Projectile(NAO_BALL,x+w/2,y+h) 
				self.balls.add(ball)
	
	def getPlayerLives(self):
		return self.player.getLives()
	
	def getBossHealth(self):
		return self.boss.getHealth()

	def getWhoKilledPlayer(self):
		return self.player.diedBy

#==================================================================
#             	         CONTROLLER
#==================================================================

class Controller:
	def __init__(self,gameModel, view):
		self.model = gameModel
		# A clock to control the FPS
		self.clock = pygame.time.Clock()

		self.player_H_Speed = 0
		self.bossTime = False
		self.previousTime = pygame.time.get_ticks()
		self.shootEvent = pygame.USEREVENT +1
		self.animateEvent = pygame.USEREVENT+2

		pygame.time.set_timer(self.shootEvent, 150)
		pygame.time.set_timer(self.animateEvent, 150)


	def updateModel(self): # Updates the model according to the events

		currentTime = pygame.time.get_ticks()

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				#======================BENDER EMOTION==================
				# FaceOrder.ChangeFace("sad2")
				# pygame.mixer.music.fadeout(2000)
				# rospy.sleep(2)
				# Talk.getInstance("oh!  you didn't like this game! ",4)
				# Talk.getInstance("let's go to the next one! ",4)
				# FaceOrder.ChangeFace("happy1")

				pygame.quit()
				quit()
			if event.type == self.shootEvent:
				self.model.spawnNaoBalls()
				pygame.time.set_timer(self.shootEvent, 150)

			if event.type == self.animateEvent:
				self.model.animateNaos()
				pygame.time.set_timer(self.animateEvent, 200)

			if event.type == KEYDOWN:
				if event.key == K_ESCAPE:
					#======================BENDER EMOTION==================
					# FaceOrder.ChangeFace("sad2")
					# pygame.mixer.music.fadeout(2000)
					# rospy.sleep(2)
					# Talk.getInstance("oh!  you didn't like this game! ",4)
					# Talk.getInstance("let's go to the next one! ",4)
					# FaceOrder.ChangeFace("happy1")

					pygame.quit()
					quit()
				if event.key == K_LEFT:
					self.player_H_Speed = -3
				if event.key == K_RIGHT:
					self.player_H_Speed = 3
				if event.key == K_SPACE:
					self.model.playerRocket()
			if event.type == pygame.KEYUP:
				if event.key == K_LEFT or event.key == K_RIGHT:
					self.player_H_Speed = 0

		if RIGHT_LEFT>0 or RIGHT_LEFT==1:
			self.player_H_Speed = -3	
		if RIGHT_LEFT<0 or RIGHT_LEFT==-1:
			self.player_H_Speed = 3
		if RIGHT_LEFT==0:
			self.player_H_Speed = 0
		if B ==1:
			self.model.playerRocket()
		if EXIT ==1:
			FaceOrder.ChangeFace("surprise")
			pygame.mixer.music.fadeout(2000)
			rospy.sleep(2)
			Talk.getInstance("oh!  you didn't like this game! ",4)
			Talk.getInstance("let's go to the next one! ",4)
			FaceOrder.ChangeFace("happy1")
			pygame.quit()
			sys.exit()

		self.model.move()
		self.model.movePlayer(self.player_H_Speed)

		self.previousTime = currentTime
		self.clock.tick(60)


	def checkGameWin(self):	
		return self.model.naosOnMap==0

	def checkGameOver(self):
		if self.model.getPlayerLives()<=0:
			return (self.model.getWhoKilledPlayer(), True)
		return (0 , False)


	def gameOverScreen(self): # It controls all the menu Screens
		rospy.Subscriber("/bender/joy/joy0", Joy, callback)

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()
			if event.type == KEYDOWN:
				if event.key == K_ESCAPE:
					pygame.quit()
					quit()
				
		if START ==1:
			print "START: ", START
			FaceOrder.ChangeFace("happy3")
			rospy.sleep(2)
			FaceOrder.ChangeFace("happy1")
			return True
        	
            # video.SetDisplay(WIDTH, HEIGHT, video.getFullscreen())
#               break
		if EXIT ==1:
			FaceOrder.ChangeFace("sad2")
			pygame.mixer.music.fadeout(2000)
			rospy.sleep(2)
			Talk.getInstance("oh!  you didn't like this game! ",4)
			Talk.getInstance("let's go to the next one! ",4)
			FaceOrder.ChangeFace("happy1")
			pygame.quit()
			sys.exit()
#==================================================================
#      		                  WINDOW
#==================================================================

class Window: # Manages the windows
	def __init__(self, gameModel, backColor):
		pygame.font.init()

		self.model = gameModel
		self.clearColor = backColor
		self.screen = None
		self.height = None
		self.width = None
		self.textFont = pygame.font.Font("freesansbold.ttf", 80)
		self.messageFont = pygame.font.Font("freesansbold.ttf", 40)
		self.liveImage = pygame.image.load(PLAYER_IMAGE)
		self.menuImages = []

		self.menuImages.append(pygame.image.load(WIN_IMAGE))
		self.menuImages.append(pygame.image.load(LOGO))
		self.menuImages.append(pygame.image.load(CONTROLS_IMAGE))
		
	def initPygame(self, width, height): # Opens the window
		pygame.init()
		self.screen = pygame.display.set_mode((width,height))
		self.height = height
		self.width = width
		pygame.display.set_caption("BenderSpace")
		
	def setFullScreen(self): # Sets the game to fullScreen
		self.screen = pygame.display.set_mode((self.width,self.height), FULLSCREEN)
		
	def drawGameUI(self): # Draws the games UI, the player lives, and bar
		lives = self.model.getPlayerLives()
		# Draw UI Bottom Line
		pygame.draw.rect(self.screen, GREEN, (0,HEIGHT-70,WIDTH,5))
		for i in range(0,lives,1):
			self.screen.blit(self.liveImage, (100+i*50,HEIGHT-60,40,40))
		
		textFont = pygame.font.Font("freesansbold.ttf", 24) #Second parameter is size
		if self.model.bossTime:
			bossHealthText = textFont.render("SuperNaoHealth", True, WHITE)
			bossRect = bossHealthText.get_rect()
			bossHealth = self.model.getBossHealth()
			barlenght = bossHealth*10
			self.screen.blit(bossHealthText,(WIDTH-bossRect.w-MOTHERSHIPHEALTH*10-40, HEIGHT-50, bossRect.w, bossRect.h))
			pygame.draw.rect(self.screen, YELLOW, (WIDTH-barlenght-20,HEIGHT-50,barlenght,40))
		
		texSurface = textFont.render("Lives:", True, WHITE)
		textRect = texSurface.get_rect()
		self.screen.blit(texSurface, (10, HEIGHT-50, textRect.w, textRect.h))
		
	def draw(self): # Draws the gameModel
		self.screen.fill(self.clearColor)
		self.model.draw(self.screen)
		
		self.drawGameUI()
		pygame.display.update()
		
	def resetModel(self, gameModel, backColor): # Resets it's internal gameModel
		self.model = gameModel
		self.clearColor = backColor
		
	def winScreen(self):
		self.screen.fill(self.clearColor)
		self.screen.blit(self.menuImages[0], (WIDTH/3,HEIGHT/3,WIDTH, HEIGHT))
		pygame.display.update()
		
	def gameOverScreen(self, causeOfDeathText):
		self.screen.fill(self.clearColor)
		gameOverText = self.textFont.render("Game Over", True, RED)
		goRect = gameOverText.get_rect()
		lostText = self.messageFont.render(causeOfDeathText, True, WHITE)
		lostRect = lostText.get_rect()
		self.screen.blit(gameOverText, (WIDTH/2-goRect.w/2, HEIGHT/3, goRect.w, goRect.h))
		self.screen.blit(lostText, ((WIDTH-lostRect.w)/2,HEIGHT*3/4-lostRect.h, lostRect.w,lostRect.h))
		
		pygame.display.update()
	
	def mainTitle(self):
		self.screen.fill(self.clearColor)
		self.screen.blit(self.menuImages[1], (WIDTH/3,0,WIDTH,HEIGHT))
		self.screen.blit(self.menuImages[2], (WIDTH/3-10, HEIGHT/3,WIDTH,HEIGHT))
		rePlayText = self.messageFont.render("Press  Green  Button  To  Begin", False, (0,255,0))
		textRect = rePlayText.get_rect()
		self.screen.blit(rePlayText, (WIDTH/2-textRect.w/2, HEIGHT-100, textRect.w, textRect.h))
		pygame.display.update()
		

#==================================================================
#   		                   MAIN
#==================================================================
def resetGame(view):
	model = Model()
	view.resetModel(model,BLACK)
	control = Controller(model, view)
	mainLoop(model, view, control)

def winScreen(view, control):
	pygame.mixer.music.stop()
	pygame.mixer.music.load(WIN_MUSIC)
	pygame.mixer.music.play(-1)
	
	view.winScreen()
	pygame.mixer.music.fadeout(5000)
	#======================BENDER EMOTION==================
	# rospy.sleep(3)
	# Talk.getInstance("did you have fun killing naos? ",4)
	# FaceOrder.ChangeFace("happy1")

	rePlay = False
	while not rePlay:
		view.winScreen()
		rePlay = control.gameOverScreen()
		
	#--If the player Hits ESCAPE this code is unreachable--
	resetGame(view) 
	
	
def gameOverScreen(view, control, diedBy):
	pygame.mixer.music.stop()
	pygame.mixer.music.load(GAMEOVER_MUSIC)
	pygame.mixer.music.play(-1)
	messageList = ["Died by Balls", "Crashed with Nao", "Naos got to the surface"]
	view.gameOverScreen(messageList[diedBy])
	pygame.mixer.music.fadeout(5000)
	#======================BENDER EMOTION==================
	# rospy.sleep(3)
	# Talk.getInstance("better luck next time! ",4)
	# rospy.sleep(1)
	# FaceOrder.ChangeFace("happy1")

	rePlay = False
	while not rePlay:
		view.gameOverScreen(messageList[diedBy])
		rePlay = control.gameOverScreen()
	#Game Ends
	
def mainLoop(model,view,control):
	pygame.mixer.music.stop()
	pygame.mixer.music.load(GAME_MUSIC)
	pygame.mixer.music.play(-1)
	
	gameOver = False
	gameWin = False

	while not gameOver:
		control.updateModel()
		view.draw()
		(diedBy , gameOver) = control.checkGameOver()
		gameWin = control.checkGameWin()
		if gameWin:
			#======================BENDER EMOTION==================
			# FaceOrder.ChangeFace("happy3")
			gameOver = True
	
	if gameWin:
		winScreen(view, control)
	else:
		gameOverScreen(view, control, diedBy)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", Joy)
    global START,RIGHT_LEFT,B,EXIT

    RIGHT_LEFT=data.axes[0]
    B = data.buttons[1]
    START = data.buttons[0]
    EXIT = data.buttons[8]


#===============================================================
#                      Main Script
#===============================================================

pygame.mixer.init()

#==================rospySubscriber=================
def getInstance():


	rospy.Subscriber("/bender/joy/joy0", Joy, callback)


	model = Model()

	color = BLACK    
	view = Window(model,color)
	view.initPygame(WIDTH, HEIGHT)
	view.setFullScreen()
		
	control = Controller(model,view)
	begin = False
	pygame.mixer.music.load(MENU_MUSIC)
	pygame.mixer.music.play(-1)

	while not begin:
		view.mainTitle()
		begin = control.gameOverScreen()

	#======================BENDER EMOTION==================
	# FaceOrder.ChangeFace("happy3")
	# rospy.sleep(3)
	# FaceOrder.ChangeFace("happy1")
	mainLoop(model,view,control)

if __name__ == '__main__':
	rospy.init_node('pyvader', anonymous=True)
	getInstance()