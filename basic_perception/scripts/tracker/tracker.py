#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from utils.filter import Tracker

class TrackingNode:
    def __init__(self, WIDTH, HEIGHT, landmarks):
        self.tracker = Tracker(WIDTH, HEIGHT, landmarks)
        self.people_sub = rospy.Subscriber("/people_locations", PoseArray, self.people_callback)
        # self.features_sub = rospy.Subscriber("/features", PoseArray, self.features_callback)
    
    def people_callback(self, detections):
        self.callback(detections, self.tracker, self.tracker.landmarks)

    # def features_callback(self, data=None):
    #     self.callback(data, self.tracker, None)

    def callback(self, detections, tracker, landmarks, features=None):
        # Change detections to numpy array        
        detections = np.array([[d.position.x, d.position.y] for d in detections.poses])
        print(detections)
        n = len(detections)
        # assert n == 1, "Number of detections must be 1"
        base_feature = np.array([i for i in range(1, n + 1)])
        features = [np.roll(base_feature, i) * 100 + np.random.randn(len(detections)) for i in range(len(detections))]
        tracker.update_tracking(detections, features, landmarks)
        print(tracker.objects)

        # Publish pose array with ids
        tracks_array = PoseArray()
        particles_array = PoseArray()
        tracks_array.header.frame_id = "map"    
        for track in tracker.objects:
            loc = track.get_estimated_loc()[0]
            t = Pose()
            t.position.x = loc[0]
            t.position.y = loc[1]
            t.orientation.w = 1
            tracks_array.poses.append(t)
            print("Tracker id:", track.obj_id, "x:", loc[0], "y:", loc[1])
            
            particles = track.get_particles() # (n, 4) : (x, y, vx, vy)
            # Guardar particulas
            for p in particles:
                t = Pose()
                t.position.x = p[0]
                t.position.y = p[1]
                t.orientation.w = 1
                particles_array.poses.append(t)
            
        # Publicar un arreglo de particles
        particles_array = PoseArray()
        particles_array.header.frame_id = "map"
        particles_pub = rospy.Publisher("/particles", PoseArray, queue_size=10)
        particles_pub.publish(particles_array)
        
        # Publicar un arreglo de tracks
        tracks_pub = rospy.Publisher("/tracks", PoseArray, queue_size=10)
        tracks_pub.publish(tracks_array)
        

def main():
    rospy.init_node('tracker', anonymous=True)
    WIDTH = 640
    HEIGHT = 736
    landmarks = np.array([[i * 100, j * 100] for i in range(6) for j in range(7)])
    # publicar landmarks para visualizarlos en rviz
    landmarks_pub = rospy.Publisher("/landmarks", PoseArray, queue_size=10)
    landmarks_array = PoseArray()
    for landmark in landmarks:
        landmark_pose = Pose()
        landmark_pose.position.x = landmark[0]
        landmark_pose.position.y = landmark[1]
        landmark_pose.orientation.w = 1
        landmarks_array.poses.append(landmark_pose)
    
    landmarks_pub.publish(landmarks_array)
    tracking_node = TrackingNode(WIDTH, HEIGHT, landmarks)
    print("Tracker node running...")
    rospy.spin()

if __name__ == '__main__':
    main()