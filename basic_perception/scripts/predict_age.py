# Import Libraries
import cv2
import os
import numpy as np

# The model architecture
# download from: https://drive.google.com/open?id=1kiusFljZc9QfcIYdU2s7xrtWHTraHwmW
AGE_MODEL = '/home/robotica/uchile_ws/ros/bender/base_ws/src/bender_core/basic_perception/scripts/weights/deploy_age.prototxt'
# The model pre-trained weights
# download from: https://drive.google.com/open?id=1kWv0AjxGSN0g31OeJa02eBGM0R_jcjIl
AGE_PROTO = '/home/robotica/uchile_ws/ros/bender/base_ws/src/bender_core/basic_perception/scripts/weights/age_net.caffemodel'
# Each Caffe Model impose the shape of the input image also image preprocessing is required like mean
# substraction to eliminate the effect of illunination changes
MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
# Represent the 8 age classes of this CNN probability layer
AGE_INTERVALS = ['(0, 2)', '(4, 6)', '(8, 12)', '(15, 20)',
                 '(25, 32)', '(38, 43)', '(48, 53)', '(60, 100)']
# Initialize frame size
frame_width = 1280
frame_height = 720
# Load age prediction model
age_net = cv2.dnn.readNet(AGE_MODEL, AGE_PROTO)

def predict_age(x,y,w,h,img_copy):
    """Predict the age of the faces showing in the image"""
    padding = 3
    face_img = img_copy[y-padding:y+h+padding,x-padding:x+w+padding]
    
    # image --> Input image to preprocess before passing it through our dnn for classification.
    blob = cv2.dnn.blobFromImage(
        image=face_img, scalefactor=1.0, size=(227, 227), 
        mean=MODEL_MEAN_VALUES, swapRB=False
    )
    # Predict Age
    age_net.setInput(blob)
    age_preds = age_net.forward()
    print("="*30, "Prediction Probabilities", "="*30)
    i = age_preds[0].argmax()
    age = AGE_INTERVALS[i]
    age_confidence_score = age_preds[0][i]
    # Draw the box
    label = "{}-{:.2f}%".format(age, age_confidence_score*100)
    print(label)
    # write the text into the frame
    
    return label