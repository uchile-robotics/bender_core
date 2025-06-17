#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from tf_transformations import euler_from_quaternion
import os
import numpy as np

class OdometryPlotter(Node):

    def __init__(self):
        super().__init__('odometry_plotter')
        self.declare_parameter('plot_filename', 'Ground_truth_trajectory.png')
        self.declare_parameter('plot_title', 'xd')
        self.declare_parameter('test_pose_label', 'AMCL')
        self.plot_filename = self.get_parameter('plot_filename').get_parameter_value().string_value
        self.plot_title = self.get_parameter('plot_title').get_parameter_value().string_value
        self.test_pose_label = self.get_parameter('test_pose_label').get_parameter_value().string_value

        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.gt_callback,
            10
        )
        self.estimation_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.estimation_callback,
            10
        )
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        ) 
        self._loop_rate = self.create_rate(10, self.get_clock())
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.plot = False
        self.ground_truth = []
        self.estimation = []

    def gt_callback(self,msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        _,_,self.theta = euler_from_quaternion(orientation)
        self.ground_truth.append([self.x, self.y, self.theta])
    def estimation_callback(self, msg:PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        _,_,self.theta = euler_from_quaternion(orientation)
        self.estimation.append([self.x, self.y, self.theta])


    def joy_callback(self,msg):  
        if msg.buttons[7] == 1 and not self.plot:
            self.plot = True 
            self.plot_odometry()
    def plot_odometry(self):
        gt_x, gt_y, gt_theta = zip(*self.ground_truth)
        est_x, est_y, est_theta = zip(*self.estimation)
        gt_x, gt_y, gt_theta = np.array(gt_x), np.array(gt_y), np.array(gt_theta)  
        est_x, est_y, est_theta = np.array(est_x), np.array(est_y), np.array(est_theta)  
        gt_orient_x = np.cos(gt_theta)
        gt_orient_y = np.sin(gt_theta)
        est_orient_x = np.cos(est_theta)
        est_orient_y = np.sin(est_theta)
        skip = 5 
        gt_skip = 200

        plt.figure(figsize=(10, 6))

        plt.plot(gt_x, gt_y, color='blue', linestyle='-', label='Trayectoria real del robot')
        plt.plot(est_x, est_y, color='red', linestyle='-', label=self.test_pose_label)

        plt.quiver(gt_x[::gt_skip], gt_y[::gt_skip],
                gt_orient_x[::gt_skip], gt_orient_y[::gt_skip],
                color='blue', scale=20, width=0.005)

        plt.quiver(est_x[::skip], est_y[::skip],
                est_orient_x[::skip], est_orient_y[::skip],
                color='red', scale=20, width=0.005)

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title(self.plot_title)
        plt.legend(loc='best')
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(self.plot_filename)
        self.get_logger().info(f'Plotted correctly, figure saved in {os.getcwd()}')
        self.plot = False

    
def main(args=None):
    rclpy.init(args=args)
    plotter = OdometryPlotter()
    plotter.get_logger().info('Node started successfully')
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
