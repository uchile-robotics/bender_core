#!/usr/bin/python

import rospy
import time

# Use HW controllers
from head_hw import HeadHW
from servos_hw import ServosHW

class EmotionsManager(object):
    """
    This is a simple interface to build complete expressions, considering eye's emotion, defined in EyeEmotion class,
    and facial gestures defined in FacialGestures class. This interface is used for ros interface.
    """
    def __init__(self, hw_controller, servos_hw):
        self.hw_controller = hw_controller
        self.servos_hw = servos_hw
        self.actual_emotion = ""
        # Check param
        self.emotions = rospy.get_param('emotions')
        self.dynamic_emotions = rospy.get_param('dynamic_emotions')
        self.colors = rospy.get_param('eye_colors')
        self.led_length = {'cheeks': 4, 'eyes': 16}

    def get_rgb_colors(self, str_colors):
        rgb_colors = []
        for color in str_colors:
            rgb_color = self.colors[color]
            rgb_colors.append(rgb_color)
        return rgb_colors

    def moveNeck(self, angle):      
        self.servos_hw.neck(angle)

    def _get_color_config(self, emo, part, side):
        colors = None
        if part in emo and side in emo[part]:
            colors = self.get_rgb_colors(emo[part][side])
        else:
            colors = self.get_rgb_colors(['black']*self.led_length[part])
        return colors

    def set_emotion(self, emotion):
        emo = self.emotions[emotion]
        # For some reason cheeks are inverted
        left_eye_colors = self._get_color_config(emo, 'eyes', 'left') + self._get_color_config(emo, 'cheeks', 'right') 
        right_eye_colors = self._get_color_config(emo, 'eyes', 'right') + self._get_color_config(emo, 'cheeks', 'left')

        self.hw_controller.set_eye_colors('left', left_eye_colors)
        self.hw_controller.set_eye_colors('right', right_eye_colors)
        if 'servos' in emo:
            self.servos_hw.left_ear(emo['servos']['left_ear'])
            self.servos_hw.right_ear(emo['servos']['right_ear'])
            self.servos_hw.left_eyebrow(emo['servos']['left_eyebrow'])
            self.servos_hw.right_eyebrow(emo['servos']['right_eyebrow'])
            self.servos_hw.mouth(emo['servos']['mouth'])

        self.actual_emotion = emotion

    def set_dynamic_emotion(self, emotion):
        emo = self.dynamic_emotions[emotion]
        if 'left_eye' in emo:
            left_eye_colors = self.get_rgb_colors(emo['left_eye'])
            self.hw_controller.set_eye_colors('left', left_eye_colors)
        if 'right_eye' in emo:
            # Use inverted
            right_eye_colors = self.get_rgb_colors(emo['right_eye'])
            self.hw_controller.set_eye_colors('right', right_eye_colors)

        max_iter = 0
        for emotion in emo:
            if emotion == 'time':
                continue
            max_iter = max(max_iter, len(emo[emotion]))

        for i in range(max_iter):
            if 'left_ear' in emo and i < len(emo['left_ear']):
                self.servos_hw.left_ear(emo['left_ear'][i])

            if 'right_ear' in emo and i < len(emo['right_ear']):
                self.servos_hw.right_ear(emo['right_ear'][i])

            if 'left_eyebrow' in emo and i < len(emo['left_eyebrow']):
                self.servos_hw.left_eyebrow(emo['left_eyebrow'][i])

            if 'right_eyebrow' in emo and i < len(emo['right_eyebrow']):
                self.servos_hw.right_eyebrow(emo['right_eyebrow'][i])

            if 'mouth' in emo and i < len(emo['mouth']):
                self.servos_hw.mouth(emo['mouth'][i])

            time.sleep(int(emo['time'])/1000.0)

    def get_state(self):
        return self.actual_emotion

