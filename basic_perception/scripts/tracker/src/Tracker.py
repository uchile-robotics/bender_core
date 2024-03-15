import scipy.spatial.distance
import scipy.stats
import numpy as np

from lapsolver import solve_dense
from src import TrackedObject
from src.Map import Map # ?

class Tracker:
    def __init__(self, width, height):
        self.objects = []
        self.id_count = 0
        self.width = width
        self.height = height
        self.MAP = Map()
        self.map, self.walkable_areas = self.MAP.load_map()
        self.map_with_margin = self.MAP.create_map_with_margin(self.map, 5)

    def get_map(self):
        return self.walkable_areas
        
    def create_obj(self, features, loc):
        obj = TrackedObject(obj_id=self.id_count, features=features, loc=loc, width=self.width, height=self.height)
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