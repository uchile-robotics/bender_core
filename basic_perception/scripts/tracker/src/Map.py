import numpy as np
from PIL import Image
import yaml
import os

class Map:
    def __init__(self):
        # mostrar directorio de este archivo
        self.this_file_path = os.path.dirname(os.path.realpath(__file__))

    def load_map(self):
        # 'stage_real.yaml'
        with open(os.path.join(self.this_file_path, 'stage_real.yaml')) as file:
            map_config = yaml.load(file, Loader=yaml.SafeLoader)

        pgm_file = map_config['image']
        resolution = map_config['resolution']
        origin = map_config['origin'][:2]

        image_pgm = Image.open(os.path.join(self.this_file_path, pgm_file))
        map = np.array(image_pgm)

        threshold_walkable = 250
        walkable_areas = map > threshold_walkable

        return map, walkable_areas

    def create_map_with_margin(self, map, padding):
        map_with_margin = np.copy(map)
        obstacle_coords = np.argwhere(map == 0)
        for x in range(len(map_with_margin)):
            for y in range(len(map_with_margin[x])):
                if map_with_margin[x][y] < 250:
                    map_with_margin[x][y] = 0

        for coord in obstacle_coords:
            x, y = coord
            for i in range(-padding, padding + 1):
                for j in range(-padding, padding + 1):
                    new_x = x + i
                    new_y = y + j
                    if 0 <= new_x < map.shape[0] and 0 <= new_y < map.shape[1]:
                        map_with_margin[new_x, new_y] = 0

        return map_with_margin