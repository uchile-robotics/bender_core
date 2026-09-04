#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Orquesta el pipeline completo: sam3 -> GraspGen -> grasp_pick_node.

Tanto sam3 como GraspGen corren como servidores HTTP en sus propios
contenedores (sam3 en :5001, GraspGen en :5002), no como nodos ROS2. En el
caso de GraspGen eso es a propósito: el host corre ROS 2 Jazzy y el
contenedor Humble, y las llamadas de servicio entre distros distintas no
interoperan (los tópicos sí, los servicios no). Con HTTP el contrato es
explícito y no depende del middleware DDS.

Este nodo expone 'run_pick_pipeline' (std_srvs/Trigger). Al llamarlo:
  1. Llama a /segment_scene (sam3_segment_node, ROS2 local) -> publica
     /sam3/object_cloud y /sam3/scene_cloud.
  2. Toma esas dos nubes y las manda por HTTP a POST /infer de GraspGen
     -> devuelve los mejores agarres, ya filtrados por colisión.
  3. Publica el mejor agarre en /grasp_candidate_pose, que es lo que ya
     escucha grasp_pick_node para planificar y ejecutar, y el conjunto
     completo en /graspgen/top_grasps para visualizar en RViz.

El callback de 'run_pick_pipeline' llama a otro servicio ROS2
(/segment_scene) y espera su respuesta. rclpy no permite anidar
spin_until_future_complete dentro de un callback que ya está siendo
procesado por el mismo executor (RuntimeError: Executor is already
spinning), por eso se usa MultiThreadedExecutor con callback groups
separados y se espera con un threading.Event.
"""
import io
import threading

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def xyz_to_npy_bytes(points: np.ndarray) -> bytes:
    buffer = io.BytesIO()
    np.save(buffer, points.astype(np.float32))
    return buffer.getvalue()


class GraspgenPickBridgeNode(Node):
    def __init__(self):
        super().__init__('graspgen_pick_bridge_node')

        self.declare_parameter('segment_service', '/segment_scene')
        self.declare_parameter('graspgen_url', 'http://192.168.1.133:5002')
        self.declare_parameter('object_cloud_topic', '/sam3/object_cloud')
        self.declare_parameter('scene_cloud_topic', '/sam3/scene_cloud')
        self.declare_parameter('service_timeout', 60.0)
        self.declare_parameter('cloud_wait', 5.0)
        self.declare_parameter('grasp_threshold', 0.6)
        self.declare_parameter('num_grasps', 200)
        self.declare_parameter('collision_threshold', 0.002)
        self.declare_parameter('top_k', 5)

        self.pipeline_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()

        self.segment_client = self.create_client(
            Trigger, self.get_parameter('segment_service').value,
            callback_group=self.client_cb_group,
        )

        self.last_object_cloud = None
        self.last_scene_cloud = None
        self.create_subscription(
            PointCloud2, self.get_parameter('object_cloud_topic').value,
            self._object_cloud_cb, 10, callback_group=self.client_cb_group,
        )
        self.create_subscription(
            PointCloud2, self.get_parameter('scene_cloud_topic').value,
            self._scene_cloud_cb, 10, callback_group=self.client_cb_group,
        )

        self.candidate_pub = self.create_publisher(PoseStamped, '/grasp_candidate_pose', 10)
        self.top_grasps_pub = self.create_publisher(PoseArray, '/graspgen/top_grasps', 10)

        self.create_service(
            Trigger, 'run_pick_pipeline', self._run_pick_pipeline_cb,
            callback_group=self.pipeline_cb_group,
        )

        self.get_logger().info(
            "graspgen_pick_bridge_node listo. servicio 'run_pick_pipeline' encadena "
            f"{self.get_parameter('segment_service').value} -> "
            f"{self.get_parameter('graspgen_url').value}/infer -> /grasp_candidate_pose"
        )

    def _object_cloud_cb(self, msg: PointCloud2):
        self.last_object_cloud = msg

    def _scene_cloud_cb(self, msg: PointCloud2):
        self.last_scene_cloud = msg

    def _call_trigger(self, client, timeout_sec):
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f"servicio {client.srv_name} no disponible"

        done_event = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done_event.set())

        if not done_event.wait(timeout_sec):
            return False, f"timeout esperando {client.srv_name}"

        result = future.result()
        if result is None:
            return False, f"{client.srv_name} no devolvió respuesta"
        return result.success, result.message

    def _run_pick_pipeline_cb(self, request, response):
        del request
        timeout = self.get_parameter('service_timeout').value

        # 1. sam3: imagen + prompt -> object_cloud + scene_cloud
        self.last_object_cloud = None
        self.last_scene_cloud = None
        ok, msg = self._call_trigger(self.segment_client, timeout)
        if not ok:
            response.success = False
            response.message = f"sam3 falló: {msg}"
            return response
        self.get_logger().info(f"sam3 OK: {msg}")

        wait = self.get_parameter('cloud_wait').value
        elapsed = 0.0
        while (self.last_object_cloud is None or self.last_scene_cloud is None) and elapsed < wait:
            threading.Event().wait(0.1)
            elapsed += 0.1

        if self.last_object_cloud is None or self.last_scene_cloud is None:
            response.success = False
            response.message = "no llegaron las nubes de sam3 a tiempo"
            return response

        object_xyz = cloud_to_xyz(self.last_object_cloud)
        scene_xyz = cloud_to_xyz(self.last_scene_cloud)
        frame_id = self.last_object_cloud.header.frame_id
        stamp = self.last_object_cloud.header.stamp

        # 2. GraspGen: nubes -> agarres libres de colisión
        try:
            reply = requests.post(
                f"{self.get_parameter('graspgen_url').value}/infer",
                files={
                    'object_cloud': ('object.npy', xyz_to_npy_bytes(object_xyz), 'application/octet-stream'),
                    'scene_cloud': ('scene.npy', xyz_to_npy_bytes(scene_xyz), 'application/octet-stream'),
                },
                data={
                    'grasp_threshold': self.get_parameter('grasp_threshold').value,
                    'num_grasps': self.get_parameter('num_grasps').value,
                    'collision_threshold': self.get_parameter('collision_threshold').value,
                    'top_k': self.get_parameter('top_k').value,
                },
                timeout=timeout,
            )
        except requests.exceptions.RequestException as exc:
            response.success = False
            response.message = f"error llamando a graspgen: {exc}"
            return response

        result = reply.json()
        if not result.get('success'):
            response.success = False
            response.message = f"graspgen: {result.get('error', 'error desconocido')}"
            return response

        # 3. Publicar el mejor agarre donde grasp_pick_node lo espera
        grasps = result['grasps']
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = stamp

        for grasp in grasps:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = grasp['position']
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = grasp['quaternion_xyzw']
            pose_array.poses.append(pose)

        best = PoseStamped()
        best.header = pose_array.header
        best.pose = pose_array.poses[0]

        self.top_grasps_pub.publish(pose_array)
        self.candidate_pub.publish(best)

        response.success = True
        response.message = (
            f"agarre publicado en /grasp_candidate_pose: score={grasps[0]['score']:.3f}, "
            f"top={result['count']}, libres={result['collision_free']}/{result['total_candidates']}, "
            f"dt={result['dt']}s"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspgenPickBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
