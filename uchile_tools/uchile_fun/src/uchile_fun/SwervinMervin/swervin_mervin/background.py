import pygame, os
import settings as s

import rospkg
rospack = rospkg.RosPack()

class Background:
    """Represents a single scrollable background in a level."""

    def __init__(self, name, parallax_speed, scale_height=False, convert=False):
        path = rospack.get_path('uchile_fun')
        path+="/src/uchile_fun/SwervinMervin/"
        self.image          = pygame.image.load(os.path.join(path+"lib", "{0}.png".format(name)))
        self.parallax_speed = parallax_speed
        self.y              = 0
        self.width          = self.image.get_width()
        self.height         = self.image.get_height()
        self.curviture      = (self.width - s.DIMENSIONS[0]) / 2
        self.scale_height   = scale_height
        self.visible_height = 0

        if convert:
            self.image = self.image.convert()

    def step(self, curve, speed_percent):
        """Moves the background one step to simulate turning."""
        c = self.curviture

        # Background is now completely off screen, so reset it.
        if c <= -self.width:
            self.curviture = c + self.width
        elif c >= self.width:
            self.curviture = (c - self.width) + (self.width - s.DIMENSIONS[0])

        self.curviture += (curve / self.parallax_speed) * speed_percent
    
    def render(self, window):
        """Draws the image to the window."""
        c = self.curviture
        w = s.DIMENSIONS[0]
        img = self.image

        # Stretch BG to fill visible area if scaling is turned on.
        if self.scale_height and self.height < self.visible_height:
            img = pygame.transform.scale(self.image, (self.width, int(self.visible_height)))

        window.blit(img,
          (0, self.y),
          (c, 0, w, s.DIMENSIONS[1]))

        # FIll empty space on the left of the screen.
        if c < 0:
            window.blit(img,
              (0, self.y),
              (self.width + c, 0, -c, s.DIMENSIONS[1]))

        # FIll empty space on the right of the screen.
        elif c > (self.width - w):
            window.blit(img,
              (self.width - c, self.y),
              (0, 0, (c - (self.width - w)), s.DIMENSIONS[1]))
