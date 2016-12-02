import cv
import cv2
import numpy as np
from PIL import Image

filename = 'tablero.png'
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

gray = np.float32(gray)
dst = cv2.cornerHarris(gray,2,3,0.04)

#result is dilated for marking the corners, not important
dst = cv2.dilate(dst,None)

resultImg = Image.new( 'RGB', (555,555), "black") # create a new black image
pixelsResultImg = resultImg.load() # create the pixel map
cornersPoints = []

for y in range(0,555):
	for x in range(0,555):
		# print x,y
		if dst[x,y]>0.01*dst.max():
			# cv.Circle(resultImg, (x,y), 2, cv.RGB(255,0,0))
			pixelsResultImg[x,y] = (255, 0, 0) # set the colour accordingly
			cornersPoints.append([x,y])
		else:
			pixelsResultImg[x,y] = (255, 255, 255) # set the colour accordingly
resultImg.show()