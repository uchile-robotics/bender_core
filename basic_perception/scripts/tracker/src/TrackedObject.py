import numpy as np
import scipy.spatial.distance
from src import ParticleFilter

class TrackedObject:
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