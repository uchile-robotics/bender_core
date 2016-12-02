import cv
import cv2
import numpy as np
from PIL import Image

cap = cv2.VideoCapture(0)
while(True):
	# Capture frame-by-frame
	ret, frame = cap.read()

	# Our operations on the frame come here
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	gray = np.float32(gray)
	dst = cv2.cornerHarris(gray,2,3,0.04)

	#result is dilated for marking the corners, not important
	dst = cv2.dilate(dst,None)
	cv2.imshow('dst',frame)

	resultImg = Image.new( 'RGB', (555,555), "black") # create a new black image
	pixelsResultImg = resultImg.load() # create the pixel map
	cornersPoints = []

	# for y in range(0,len(dst)):
	# 	for x in range(0,len(dst)):
	# 		# print x,y
	# 		if dst[x,y]>0.01*dst.max():
	# 			# cv.Circle(resultImg, (x,y), 2, cv.RGB(255,0,0))
	# 			pixelsResultImg[x,y] = (255, 0, 0) # set the colour accordingly
	# 			cornersPoints.append([x,y])
	# 		else:
	# 			pixelsResultImg[x,y] = (255, 255, 255) # set the colour accordingly
	# resultImg.show()
	frame[dst>0.01*dst.max()]=[255,0,0]
	cv2.imshow('dst',frame)
	# cv2.imshow('frame',gray)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()