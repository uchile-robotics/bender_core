import numpy as np
from numpy.random import uniform


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