#!/usr/bin/env python3.9
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, ColorRGBA
from geometry_msgs.msg import PoseArray, Pose, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
import colorsys
from scripts.tracker.utils.filter import Tracker
import pickle


def loc_to_pixel(loc, resolution=0.02, origin=(-3, -5)):
    return (loc - np.array(origin)) / resolution

def pixel_to_loc(pixel, resolution=0.02, origin=(-3, -5)):
    return pixel * resolution + np.array(origin)

class TrackingNode:
    def __init__(self, WIDTH, HEIGHT):
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.tracker = Tracker(WIDTH, HEIGHT)
        self.people_sub = rospy.Subscriber("/people_detections", Float64MultiArray, self.callback)
        self.particles_pub = rospy.Publisher("/particle_cloud", PoseArray, queue_size=10)
        self.map_pub = rospy.Publisher("/tracker_map", OccupancyGrid, queue_size=10)
        self.markers_pub = rospy.Publisher("/markers", Marker, queue_size=10)
        self.rect_pub = rospy.Publisher("/rect", Marker, queue_size=10)
        self.historical_data = {'detections': [], 'features': []}  # Initialize an empty dictionary

    def save_historical_data(self):
        with open('historical_data.pkl', 'wb') as f:
            pickle.dump(self.historical_data, f)

    def callback(self, data):
        # Obtener mapa del tracker
        map = self.tracker.get_map()
        # Crear mensaje de mapa
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = 0.05
        map_msg.info.width = map.shape[1]
        map_msg.info.height = map.shape[0]
        map_msg.info.origin.position.x = -map.shape[1] * map_msg.info.resolution / 2
        map_msg.info.origin.position.y = -map.shape[0] * map_msg.info.resolution / 2
        map_msg.data = map.flatten()
        self.map_pub.publish(map_msg)
        
        
                
        bounding_box = [
            [0, 0], # Starting point
            [self.WIDTH, 0], # Top right corner
            [self.WIDTH, self.HEIGHT], # Bottom right corner
            [0, self.HEIGHT], # Bottom left corner
            [0, 0],  # Closing the loop
        ]
        
        bounding_box = np.array(bounding_box)
        bounding_box = bounding_box*0.02 + np.array([-3, -5])

        # Create a visualization marker for the bounding box
        bounding_box_marker = Marker()
        bounding_box_marker.header.frame_id = "map"
        bounding_box_marker.type = Marker.LINE_STRIP
        bounding_box_marker.action = Marker.ADD
        bounding_box_marker.scale.x = 0.05  # Adjust the line width
        bounding_box_marker.color.r = 1.0
        bounding_box_marker.color.g = 0.0
        bounding_box_marker.color.b = 0.0
        bounding_box_marker.color.a = 1.0

        # Convert the bounding box coordinates to ROS Point messages
        for point in bounding_box:
            p = Point()
            p.x, p.y = point
            bounding_box_marker.points.append(p)

        # Publish the bounding box marker
        self.rect_pub.publish(bounding_box_marker)

        # Printear dimensiones de las detecciones (1026*n) donde n es el numero de detecciones

        # Hacer reshape para tener (n, 1026)
        data = np.array(data.data).reshape(-1, 1026)
        # Las detecciones son una lista de [x, y]
        detections = [d[:2] for d in data]
        # Las features son una lista de [f1, f2, ..., fn]
        features = [d[2:] for d in data]

        self.historical_data['detections'].append(detections)
        self.historical_data['features'].append(features)

        # Obtener indices de detecciones nan en x o y (a veces pasa)
        nan_pos_indices = []
        for i, d in enumerate(detections):
            if np.isnan(d[0]) or np.isnan(d[1]):
                nan_pos_indices.append(i)
        
        # Borrar de detections y features las detecciones nan
        detections = np.delete(detections, nan_pos_indices, axis=0)
        features = np.delete(features, nan_pos_indices, axis=0)

        # Transformar detecciones al mapa de pixeles
        if len(detections) > 0: detections = loc_to_pixel(detections)

        print("Detections:", detections)
        print("Features:", features)

        self.tracker.update_tracking(detections, features)
        self.tracker.print_objs()

        # Publish markers with ids
        markers_array = Marker()
        markers_array.header.frame_id = "map"
        markers_array.type = Marker.POINTS
        markers_array.action = Marker.ADD
        markers_array.scale.x = 0.2  # Tamaño del marcador en el eje x
        markers_array.scale.y = 0.2  # Tamaño del marcador en el eje y
        markers_array.color.a = 1.0  # Transparencia
        
        # Crear el mensaje de la nube de puntos
        particle_cloud_msg = PoseArray()
        particle_cloud_msg.header.frame_id = "map"
        

        for i, track in enumerate(self.tracker.objects):
            # Transformar posicion estimada al mapa del mundo
            loc = pixel_to_loc(track.get_estimated_loc()[0])
            p = Point()
            p.x = loc[0]
            p.y = loc[1]
            markers_array.points.append(p)

            # Asignar un color único basado en el ID
            hue = (i % 10) / 10.0  # Asegura que el tono está en el rango [0, 1]
            rgb = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

            # Corrige el formato del color para que sea un objeto ColorRGBA
            color = {'r': rgb[0], 'g': rgb[1], 'b': rgb[2], 'a': 1.0}
            markers_array.colors.append(ColorRGBA(**color))
 
            # Obtener las particulas del tracker
            particles = track.get_particles()
            # Guardar particulas
            for particle in particles:
                t = Pose()
                pos = pixel_to_loc(particle[:2])
                t.position.x = pos[0]
                t.position.y = pos[1]
                t.orientation.w = 1
                particle_cloud_msg.poses.append(t)

        # Publicar nube de puntos
        self.markers_pub.publish(markers_array)
        self.particles_pub.publish(particle_cloud_msg)

def main():
    rospy.init_node('tracker', anonymous=True)
    WIDTH = 640
    HEIGHT = 736
    tracking_node = TrackingNode(WIDTH, HEIGHT)
    print("Tracker node waiting for people locations...")
    rospy.spin() # Handle Ctrl+C interruption
    print("Saving tracker state...")
    tracking_node.save_historical_data()
    print("Tracker state saved.")

if __name__ == '__main__':
    main()