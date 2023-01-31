# Import Libraries
import cv2
import numpy as np

# The gender model architecture
# https://drive.google.com/open?id=1W_moLzMlGiELyPxWiYQJ9KFaXroQ_NFQ
GENDER_MODEL = '/home/robotica/uchile_ws/ros/bender/base_ws/src/bender_core/basic_perception/scripts/weights/deploy_gender.prototxt'
# The gender model pre-trained weights
# https://drive.google.com/open?id=1AW3WduLk1haTVAxHOkVS_BEzel1WXQHP
GENDER_PROTO = '/home/robotica/uchile_ws/ros/bender/base_ws/src/bender_core/basic_perception/scripts/weights/gender_net.caffemodel'
# Each Caffe Model impose the shape of the input image also image preprocessing is required like mean
# substraction to eliminate the effect of illunination changes
MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
# Represent the gender classes
GENDER_LIST = ['Male', 'Female']
# load face Caffe model
# Load gender prediction model
gender_net = cv2.dnn.readNetFromCaffe(GENDER_MODEL, GENDER_PROTO)

def predict_gender(x,y,w,h,img_copy):
    """Predict the gender of the faces showing in the image"""
    padding = 3
    face_img = img_copy[y-padding:y+h+padding,x-padding:x+w+padding]
    
    blob = cv2.dnn.blobFromImage(image=face_img, scalefactor=1.0, size=(
        227, 227), mean=MODEL_MEAN_VALUES, swapRB=False, crop=False)
    # Predict Gender
    gender_net.setInput(blob)
    gender_preds = gender_net.forward()
    i = gender_preds[0].argmax()
    gender = GENDER_LIST[i]
    gender_confidence_score = gender_preds[0][i]
    # Draw the box
    label = "{}-{:.2f}%".format(gender, gender_confidence_score*100)
    print(label)
    # get the font scale for this image size
    box_color = (255, 0, 0) if gender == "Male" else (147, 20, 255)
    # uncomment if you want to save the image
    # cv2.imwrite("output.jpg", frame)
    # Cleanup
    return label