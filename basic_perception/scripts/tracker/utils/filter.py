#!/usr/bin/env python3
import numpy as np
from numpy.random import uniform
import scipy.stats
from PIL import Image
from lapsolver import solve_dense
import yaml
import sys
import os

# mostrar directorio de este archivo
this_file_path = os.path.dirname(os.path.realpath(__file__))

def load_map():
    # 'stage_real.yaml'
    with open(os.path.join(this_file_path, 'stage_real.yaml')) as file:
        map_config = yaml.load(file, Loader=yaml.SafeLoader)

    pgm_file = map_config['image']
    resolution = map_config['resolution']
    origin = map_config['origin'][:2]

    image_pgm = Image.open(os.path.join(this_file_path, pgm_file))
    map = np.array(image_pgm)

    threshold_walkable = 250
    walkable_areas = map > threshold_walkable

    return map, walkable_areas

def create_map_with_margin(map, padding):
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

class ParticleFilter:
    def __init__(self, x_range, y_range, N, sensor_std_err=0.02, init_loc=None, width=640, height=736, origin=[0, 0]):
        self.sensor_std_err = sensor_std_err
        self.N = N
        self.std_xy = np.array([width, height]) / (width * height) * 0.1
        self.std_hd = np.array([np.pi/2, 1000.0 * max(width, height) / (width * height)])
        self.std_axy = np.array([width, height]) / (width * height) * 0.5
        if init_loc is None:
            self.particles = self.create_uniform_particles(x_range, y_range)
        else:
            self.particles = self.create_gaussian_particles(mean=np.array([init_loc[0], init_loc[1]]), std=self.std_xy)
        self.weights = np.array([1.0] * self.N)
        self.center = np.array([[init_loc[0], init_loc[1]]])
        self.trajectory = np.zeros(shape=(0, 2))
        self.estimated_trajectory = np.zeros(shape=(0, 2))
        self.estimated_var = np.zeros(shape=(0, 2))
        self.robot_pos = np.zeros(shape=(0, 2))
        self.previous_x = -1
        self.previous_y = -1
        self.zs = None
        self.width = width
        self.height = height
        self.origin = origin

    def create_uniform_particles(self, x_range, y_range):
        particles = np.empty((self.N, 4))
        particles[:, 0] = uniform(x_range[0], x_range[1], size=self.N)
        particles[:, 1] = uniform(y_range[0], y_range[1], size=self.N)
        particles[:, 2] = 0.0  # Initial velocity in x
        particles[:, 3] = 0.0  # Initial velocity in y
        return particles
    
    def create_gaussian_particles(self, mean, std):
        particles = np.empty((self.N, 4))
        particles[:, 0] = mean[0] + (np.random.randn(self.N) * std[0])
        particles[:, 1] = mean[1] + (np.random.randn(self.N) * std[1])
        particles[:, 2] = 0.0  # Initial velocity in x
        particles[:, 3] = 0.0  # Initial velocity in y
        return particles
    
    def rutine(self, x = None, y = None):
        obs = True if x is not None and y is not None else False
        if obs: self.center = np.array([[x, y]])
        # Agregar la posicion del robot a la trayectoria (si no hay, se agrega el centro previo)
        self.trajectory = np.vstack((self.trajectory, self.center))
        if self.previous_x > 0: # Si existe una medicion, actualizar movimiento
            self.motion_model([4, 0.0001], dt=1.)
            if obs:  # Si hay medicion, actualizar movimiento y pesos
                self.update_weights(self.center, self.sensor_std_err)
                if self.neff() < 0.8 * self.N:
                    indexes = self.systematic_resample()
                    self.resample_from_index(indexes)
        # Agregar la estimacion a la trayectoria estimada
        self.estimated_trajectory = np.vstack((self.estimated_trajectory, self.estimate()[0]))
        self.estimated_var = np.vstack((self.estimated_var, self.estimate()[1]))
        self.previous_x = self.center[0, 0]
        self.previous_y = self.center[0, 1]

    def motion_model(self, motion_noise, dt = 1.):
        pos_noise = [motion_noise[0], motion_noise[0]]
        vel_noise = [motion_noise[1], motion_noise[1]]
        self.particles[:, :2] += self.particles[:, 2:4] * dt # Move
        self.particles[:, :2] += np.random.normal(0, pos_noise, self.particles[:, :2].shape) * dt # Add noise (x, y)
        self.particles[:, 2:4] += np.random.normal(0, vel_noise, self.particles[:, 2:4].shape) * dt # Add noise (vx, vy)
        self.particles[:, :2] = np.clip(self.particles[:, :2], 0, np.array([self.width, self.height])) # Clip to map size

    def update_weights(self, measurements, measurement_noise):
        # # measurement_noise
        likelihood = np.exp(-0.5 * ((self.particles[:, :2] - measurements) / 1) ** 2)
        self.weights *= np.prod(likelihood, axis=1)
        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= self.weights.sum()

    def neff(self):
        return 1. / np.sum(np.square(self.weights))

    def systematic_resample(self):
        positions = (np.arange(self.N) + np.random.random()) / self.N
        indexes = np.zeros(self.N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < self.N and j < self.N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes
                    
    def estimate(self):
        pos = self.particles[:, 0:2]
        mean = np.average(pos, weights=self.weights, axis=0)
        var = np.average((pos - mean) ** 2, weights=self.weights, axis=0)
        return mean, var

    def resample_from_index(self, indexes):
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights /= np.sum(self.weights)
        
        
class Obj:
    def __init__(self, obj_id, features, loc, width, height, x_range=[0, 640], y_range=[0, 736], N=100, sensor_std_err=0.1):
        self.obj_id = obj_id
        self.features = features
        self.loc = loc
        self.delta_features = np.zeros(self.features.shape)
        self.delta_loc = np.zeros(self.loc.shape)
        self.pf = ParticleFilter(x_range=x_range, y_range=y_range, N=N, sensor_std_err=sensor_std_err, init_loc=loc)
        self.update(features, loc)
        self.sleep_time = 0
        self.width = width
        self.height = height
        
    def update_features(self, features):
        if features is not None:
            self.delta_features = scipy.spatial.distance.cosine(self.features, features) # np.linalg.norm(self.features - features)
            alpha = 0.1
            self.features = (1-alpha) * self.features + alpha * features
            self.sleep_time = 0
        else:
            self.sleep_time += 1
        
    def update(self, features = None, loc = None):
        self.update_features(features)
        self.update_loc(loc)
        
    def update_loc(self, loc):
        if loc is None:
            self.pf.rutine()
        else:
            # self.delta_loc = np.linalg.norm(self.loc - loc)
            self.delta_loc = scipy.spatial.distance.cosine(self.loc, loc)
            self.loc = loc
            self.pf.rutine(loc[0], loc[1])
        
    def get_estimated_loc(self):
        return self.pf.estimate()
    
    def get_particles(self):
        return self.pf.particles

    def __str__(self):
        
        data = [
            self.obj_id,
            np.round(self.features, 2),
            self.delta_features,
            np.round(self.loc, 2),
            self.delta_loc,
            np.round(self.pf.estimate()[0], 2),
            np.round(self.pf.estimate()[1], 2)
        ]
        
        std_output = """
        Obj ID: {}\n
        Features: {} \t\t Norm diff: {}\n
        Loc: {} \t\t Norm diff: {}\n
        Estimated loc: {}\n
        Var: {}
        """.format(*data)
        return std_output
    
class Tracker:
    def __init__(self, width, height):
        self.objects = []
        self.id_count = 0
        self.width = width
        self.height = height
        self.map, self.walkable_areas = load_map()
        self.map_with_margin = create_map_with_margin(self.map, padding=5)

    def get_map(self):
        return self.walkable_areas
        
    def create_obj(self, features, loc):
        obj = Obj(obj_id=self.id_count, features=features, loc=loc, width=self.width, height=self.height)
        self.objects.append(obj)
        self.id_count += 1
        
    def normalized_mahalanobis(self, x, mean, cov):
        # Chi-squared distribution with 2 degrees of freedom
        mahalanobis = scipy.spatial.distance.mahalanobis(x, mean, np.diag(cov))
        return 1 - scipy.stats.chi2.cdf(mahalanobis ** 2, 2)

    def update_tracking(self, dets, features):  
        # Assuming dets is a list of [x, y] coordinates and features is a list of feature vectors
        detections = np.array(dets)
        features = np.array(features)

        # If no detections, update all objects with no detections
        if len(detections) == 0:
            for obj in self.objects:
                obj.update()
            return

        # If no objects, create new objects for each detection
        if not self.objects:
            for i, det in enumerate(detections):
                self.create_obj(features[i], det)
        else:
            # Perform data association using the Hungarian algorithm
            cost_matrix = np.zeros((len(self.objects), len(detections)))
            for i, obj in enumerate(self.objects):
                for j, det in enumerate(detections):
                    # Costo es la distancia entre las features y la de Mahalanobis normalizada para la posicion
                    f_cost = scipy.spatial.distance.cosine(obj.features, features[j])
                    maha_cost = self.normalized_mahalanobis(det, *obj.pf.estimate())
                    cost_matrix[i, j] = (0.95*f_cost + 0.05*maha_cost)
                    print(f"F. cost: {f_cost:.3f} \t\t MH. cost: {maha_cost:.3f} \t\t Cost pre-clip: {cost_matrix[i, j]:.3f}")
                    if cost_matrix[i, j] > 0.55: 
                        cost_matrix[i, j] = np.nan
                    # elif maha_cost < 0.01: # Si el track esta muy cerca de la deteccion no usar la distancia de features
                    #     cost_matrix[i, j] = (0.2*f_cost + 0.8*maha_cost*0)
            
            # row_ind, col_ind = linear_sum_assignment(cost_matrix) # Usar solver de lapsolver
            row_ind, col_ind = solve_dense(cost_matrix)

            # Update existing objects with new detections
            for i, obj in enumerate(self.objects):
                if i in row_ind:
                    j = row_ind.tolist().index(i)
                    obj.update(features[j], detections[j])
                else:
                    obj.update()

            # Create new objects for unmatched detections
            unmatched_dets = set(range(len(detections))) - set(col_ind)
            for j in unmatched_dets:
                self.create_obj(features[j], detections[j])

        # Print the updated objects
        # self.print_objs()

    def print_objs(self):
        print("-" * 30)
        for obj in self.objects:
            print(obj)
        print("-" * 30)

map, walkable_area = load_map()
map_with_margin = create_map_with_margin(map, padding=5)
map_with_margin = np.flip(map_with_margin, axis=0)

# Example usage
if __name__ == "__main__":
    from time import sleep
    import pickle
    import cv2
    # Example usage for the particle filter visualization
    # resolution = 0.05
    resolution = 1
    WIDTH = 640 * resolution
    HEIGHT = 736 * resolution
    WINDOW_NAME = "Particle Filter"
    DELAY_MSEC = 30

    data = pickle.load(open(os.path.join(this_file_path, '../input/real_example.pkl'), 'rb'), encoding='latin1')
    locs = data['detections']
    feats = data['features']

    # Borrar primeros 20 frames
    delete_frames = 300
    amp = 1

    # Update the tracker with detections and features
    step = 0
    # Create a Tracker instance
    tracker = Tracker(WIDTH, HEIGHT)
    print(map_with_margin)
    
    while True:
        if step == 1:
            sleep(5)
        try:
            # Amplificar detecciones
            detections = locs[step]#, locs2[step] * amp, locs3[step] * amp ]
            features = feats[step]#, feats2[step], feats3[step]]
            
            # Obtener indices de detecciones nan en x o y (a veces pasa)
            nan_pos_indices = []
            for i, d in enumerate(detections):
                if np.isnan(d[0]) or np.isnan(d[1]):
                    nan_pos_indices.append(i)
            
            # Borrar de detections y features las detecciones nan
            detections = np.delete(detections, nan_pos_indices, axis=0)
            # Transformar + (3, 5) para que quede en el mapa * 0.02
            if len(detections) > 0:
                detections = (detections + np.array([3, 5]) )/ 0.02 
            
            features = np.delete(features, nan_pos_indices, axis=0)
            
            # Filtrar las detecciones junto a sus features que esten en un obstaculo
            obstacle_indices = []
            for i, d in enumerate(detections):
                x, y = int(d[0]), int(d[1])
                map_t = map_with_margin.T
                if map_t[x, y] < 1:
                    obstacle_indices.append(i)
            detections = np.delete(detections, obstacle_indices, axis=0)
            features = np.delete(features, obstacle_indices, axis=0)
            
            tracker.update_tracking(detections, features)
            tracker.print_objs()
            step += 1
        # Quedarse en el ultimo frame
        except Exception as e:
            print('Error: ', e)
        
        # Visualize the particle filter
        img = map_with_margin
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img = cv2.resize(img, (WIDTH, HEIGHT))
        
        # Graficar detecciones
        step2 = 0
        while step2 < step:
            colors = ["#3396FF", "#CE33FF", "#FF8A33"]
            for idx, loc in enumerate(locs[step2]):
                # Verificar que no sea NaN
                if not np.isnan(loc[0]):
                    # Dibujar un punto
                    loc = (loc + np.array([3, 5])) / 0.02
                    # Verificar que este en el mapa
                    x, y = int(loc[0]), int(loc[1])
                    map_t = map_with_margin.T
                    if map_t[x, y] < 1:
                        continue
                    color = tuple(int(colors[idx][i:i+2], 16) for i in (1, 3, 5))
                    # Agregar un punto grande
                    cv2.circle(img, tuple(loc.astype(int)), 5, color, -1)
            step2 += 1

        # Poner un color a cada objeto con sus particulas segun su id (color = f(id))
        max_ids = 3.0
        for obj in tracker.objects:
            if obj.obj_id < max_ids:
                colors = ["#3396FF", "#CE33FF", "#FF8A33"]
                color_hex = colors[obj.obj_id]
                color = tuple(int(color_hex[i:i+2], 16) for i in (1, 3, 5))
                for particle in obj.pf.particles:
                    # Opacidad de las particulas baja
                    cv2.circle(img, (int(particle[0]), int(particle[1])), 1, color, -1, cv2.LINE_AA)
                # Agregar una flecha que indique la velocidad media en el centro
                mean_vel = np.mean(obj.pf.particles[:, 2:4], axis=0)
                    
            cv2.putText(img, str(obj.obj_id), (int(obj.pf.center[0, 0]), int(obj.pf.center[0, 1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 0, 0), 1, cv2.LINE_AA)
            # Dibujar la trayectoria si tiene mas de 2 puntos
            for i in range(max(2, obj.pf.trajectory.shape[0] - 100), obj.pf.trajectory.shape[0] - 1):
                # Dibujar la trayectoria estimada si tiene mas de 2 puntos (en escala de rojo a verde dependiendo de la varianza)
                if obj.pf.estimated_trajectory.shape[0] > 2:
                    mean_var = (obj.pf.estimated_var[i, 0] + obj.pf.estimated_var[i, 1]) * 0.5
                    cv2.line(img, (int(obj.pf.estimated_trajectory[i, 0]), int(obj.pf.estimated_trajectory[i, 1])), (int(obj.pf.estimated_trajectory[i + 1, 0]), int(obj.pf.estimated_trajectory[i + 1, 1])), (int(255 * (mean_var/ 200)), int(255 * (1 - mean_var/ 200)), 0), 2)
                if obj.pf.trajectory.shape[0] > 2:
                    trajectory_color = "#DF99E2"
                    trajectory_color = tuple(int(trajectory_color[i:i+2], 16) for i in (1, 3, 5))
                    cv2.line(img, (int(obj.pf.trajectory[i, 0]), int(obj.pf.trajectory[i, 1])), (int(obj.pf.trajectory[i + 1, 0]), int(obj.pf.trajectory[i + 1, 1])), trajectory_color, 2)
        
        # Cambiar canales de color
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Agrandar la imagen
        cv2.imshow(WINDOW_NAME, img)
        key = cv2.waitKey(DELAY_MSEC)

        # Press 'q' to exit
        if key == ord('q'):
            break
    cv2.destroyAllWindows()