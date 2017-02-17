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
        self.led_colors = dict()
        self.led_colors['eyes'] = {'left' : ['black']*16, 'right' : ['black']*16} 
        self.led_colors['cheeks'] = {'left' : ['black']*4, 'right' : ['black']*4} 
        self.led_length = {'cheeks': 4, 'eyes': 16}

    def get_rgb_colors(self, str_colors):
        rgb_colors = []
        for color in str_colors:
            rgb_color = self.colors[color]
            rgb_colors.append(rgb_color)
        return rgb_colors

    def _get_color_config(self, emo, part, side):
        colors = None
        if part in emo and side in emo[part]:
            colors = emo[part][side]
        else:
            colors = ['black']*self.led_length[part]
        return colors

    def set_emotion(self, emotion):
        emo = self.emotions[emotion]
        self.led_colors['eyes']['left']     = self._get_color_config(emo, 'eyes', 'left')
        self.led_colors['cheeks']['left']   = self._get_color_config(emo, 'cheeks', 'left')
        self.led_colors['eyes']['right']    = self._get_color_config(emo, 'eyes', 'right')
        self.led_colors['cheeks']['right']  = self._get_color_config(emo, 'cheeks', 'right')
        # Send command, for some reason cheeks are inverted
        self.hw_controller.set_eye_colors('left', self.get_rgb_colors(self.led_colors['eyes']['left'] + self.led_colors['cheeks']['right']))
        self.hw_controller.set_eye_colors('right', self.get_rgb_colors(self.led_colors['eyes']['right'] + self.led_colors['cheeks']['left']))
        if 'servos' in emo:
            self.servos_hw.left_ear(emo['servos']['left_ear'])
            self.servos_hw.right_ear(emo['servos']['right_ear'])
            self.servos_hw.left_eyebrow(emo['servos']['left_eyebrow'])
            self.servos_hw.right_eyebrow(emo['servos']['right_eyebrow'])
            self.servos_hw.mouth(emo['servos']['mouth'])

        self.actual_emotion = emotion

    def _get_update_color_config(self, emo, part, side, idx=0):
        colors = None
        if part in emo and side in emo[part] and idx < len(emo[part][side]):
            colors = emo[part][side][idx]
        else:
            colors = self.led_colors[part][side]
        return colors

    def set_dynamic_emotion(self, emotion):
        emo = self.dynamic_emotions[emotion]
        
        # Find largest array
        max_iter = 0
        for part in emo:
            if part == 'time':
                continue
            for element in emo[part]:
                max_iter = max(max_iter, len(emo[part][element]))


        for i in range(max_iter):
            if 'servos' in emo:
                servos = emo['servos']
                if 'left_ear' in servos and i < len(servos['left_ear']):
                    self.servos_hw.left_ear(servos['left_ear'][i])

                if 'right_ear' in servos and i < len(servos['right_ear']):
                    self.servos_hw.right_ear(servos['right_ear'][i])

                if 'left_eyebrow' in servos and i < len(servos['left_eyebrow']):
                    self.servos_hw.left_eyebrow(servos['left_eyebrow'][i])

                if 'right_eyebrow' in servos and i < len(servos['right_eyebrow']):
                    self.servos_hw.right_eyebrow(servos['right_eyebrow'][i])

                if 'mouth' in servos and i < len(servos['mouth']):
                    self.servos_hw.mouth(servos['mouth'][i])

            # Only publish when cmd change with respect to current configuration
            left_eyes_cmd = self._get_update_color_config(emo, 'eyes', 'left', i)
            left_cheek_cmd = self._get_update_color_config(emo, 'cheeks', 'left', i)
            right_eyes_cmd = self._get_update_color_config(emo, 'eyes', 'right', i)
            right_cheek_cmd = self._get_update_color_config(emo, 'cheeks', 'right', i)

            if left_eyes_cmd != self.led_colors['eyes']['left'] or right_cheek_cmd != self.led_colors['cheeks']['right']:
                self.led_colors['eyes']['left'] = left_eyes_cmd
                self.led_colors['cheeks']['right'] = right_cheek_cmd
                # Send command, for some reason cheeks are inverted
                self.hw_controller.set_eye_colors('left', self.get_rgb_colors(self.led_colors['eyes']['left'] + self.led_colors['cheeks']['right']))
            
            if right_eyes_cmd != self.led_colors['eyes']['right'] or right_cheek_cmd != self.led_colors['cheeks']['left']:
                self.led_colors['eyes']['right'] = right_eyes_cmd
                self.led_colors['cheeks']['left'] = left_cheek_cmd
                # Send command, for some reason cheeks are inverted
                self.hw_controller.set_eye_colors('right', self.get_rgb_colors(self.led_colors['eyes']['right'] + self.led_colors['cheeks']['left']))

            time.sleep(int(emo['time'])/1000.0)

    def get_state(self):
        return self.actual_emotion

