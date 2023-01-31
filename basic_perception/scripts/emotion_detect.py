#!/usr/bin/env python
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import bleedfacedetector as fd
import time

def init_emotion(model):
    
    # Set global variables
    global net,emotions
    
    # Define the emotions
    emotions = ['Neutral', 'Happy', 'Surprise', 'Sad', 'Anger', 'Disgust', 'Fear', 'Contempt']
    
    # Initialize the DNN module
    net = cv2.dnn.readNetFromONNX(model)


def emotion(image, returndata=True):
    # Make copy of  image
    img_copy = image.copy()
    
    # Detect faces in image
    faces = fd.ssd_detect(img_copy,conf=0.2)
    
    # Define padding for face ROI
    padding = 3 
    
    emo = "Neutral"
    # Iterate process for all detected faces
    if faces == []:
      return image , emo
    else:
        face_close = faces[0] 
        x = face_close[0]
        y = face_close[1]
        w = face_close[2]
        h = face_close[3]
        # Get the Face from image
        face = img_copy[y-padding:y+h+padding,x-padding:x+w+padding]
        
        # Convert the detected face from BGR to Gray scale
        gray = cv2.cvtColor(face,cv2.COLOR_BGR2GRAY)
        
        # Resize the gray scale image into 64x64
        resized_face = cv2.resize(gray, (64, 64))
        
        # Reshape the final image in required format of model
        processed_face = resized_face.reshape(1,1,64,64)
        
        # Input the processed image
        net.setInput(processed_face)
        
        # Forward pass
        Output = net.forward()
 
        # Compute softmax values for each sets of scores  
        expanded = np.exp(Output - np.max(Output))
        probablities =  expanded / expanded.sum()
        
        # Get the final probablities by getting rid of any extra dimensions 
        prob = np.squeeze(probablities)
        
        # Get the predicted emotion
        predicted_emotion = emotions[prob.argmax()]
        print(predicted_emotion)
       
        # Write predicted emotion on image
        cv2.putText(img_copy,'{}'.format(predicted_emotion),(x,y+h+(1*20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 
                        2, cv2.LINE_AA)
        # Draw a rectangular box on the detected face
        cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,0,255),2)

        emo = predicted_emotion

        return img_copy, predicted_emotion
