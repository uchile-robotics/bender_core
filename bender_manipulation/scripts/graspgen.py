#!/usr/bin/env python3
"""Nodo GraspGen: de nube del objeto + nube de la escena a poses de agarre.

Este nodo expone 'generate_grasps' (uchile_srvs/GraspGen). Al llamarlo:
  Toma las dos nubes de la request (object_cloud y scene_cloud, por ejemplo
  la respuesta de 'segment_scene') y las manda por HTTP a POST /infer de
  GraspGen -> devuelve los mejores agarres, ya filtrados por colisión.

La respuesta trae los agarres ordenados por score (top_grasps) y el mejor
por separado (best_pose), en el frame de object_cloud. También se publican
en /graspgen/top_grasps y /graspgen/best_grasp para visualizar en RViz.

"""

import io

import numpy as np
import rclpy
import requests
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from uchile_srvs.srv import GraspGen


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points_numpy(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )
    return np.asarray(points, dtype=np.float32).reshape(-1, 3)


def xyz_to_npy_bytes(points: np.ndarray) -> bytes:
    buffer = io.BytesIO()
    np.save(buffer, points.astype(np.float32))
    return buffer.getvalue()


class GraspgenNode(Node):
    def __init__(self):
        super().__init__("graspgen_node")

        self.declare_parameter("graspgen_url", "http://tesistas-MS-7E12.local:5002")
        self.declare_parameter("request_timeout", 60.0)
        self.declare_parameter("grasp_threshold", 0.6)
        self.declare_parameter("num_grasps", 200)
        self.declare_parameter("collision_threshold", 0.002)
        self.declare_parameter("top_k", 5)

        # same grasps as the service response, published for RViz
        self.top_grasps_pub = self.create_publisher(
            PoseArray, "/graspgen/top_grasps", 10
        )
        self.best_grasp_pub = self.create_publisher(
            PoseStamped, "/graspgen/best_grasp", 10
        )

        self.create_service(GraspGen, "generate_grasps", self._generate_grasps_cb)

        self.get_logger().info(
            "graspgen_node listo. servicio 'generate_grasps' -> "
            f"{self.get_parameter('graspgen_url').value}/infer"
        )

    @staticmethod
    def _fail(response: GraspGen.Response, message: str):
        # best_pose and top_grasps stay empty
        response.success = False
        response.message = message
        return response

    def _infer(self, object_xyz: np.ndarray, scene_xyz: np.ndarray) -> dict:
        """Llama a GraspGen /infer. Devuelve el JSON de la respuesta, o levanta
        requests.exceptions.RequestException / ValueError si falla la llamada."""
        reply = requests.post(
            f"{self.get_parameter('graspgen_url').value}/infer",
            files={
                "object_cloud": (
                    "object.npy",
                    xyz_to_npy_bytes(object_xyz),
                    "application/octet-stream",
                ),
                "scene_cloud": (
                    "scene.npy",
                    xyz_to_npy_bytes(scene_xyz),
                    "application/octet-stream",
                ),
            },
            data={
                "grasp_threshold": self.get_parameter("grasp_threshold").value,
                "num_grasps": self.get_parameter("num_grasps").value,
                "collision_threshold": self.get_parameter("collision_threshold").value,
                "top_k": self.get_parameter("top_k").value,
            },
            timeout=self.get_parameter("request_timeout").value,
        )
        reply.raise_for_status()
        return reply.json()

    def _generate_grasps_cb(
        self, request: GraspGen.Request, response: GraspGen.Response
    ):
        object_xyz = cloud_to_xyz(request.object_cloud)
        scene_xyz = cloud_to_xyz(request.scene_cloud)
        if object_xyz.shape[0] == 0:
            return self._fail(response, "object_cloud is empty")

        # GraspGen: nubes -> agarres libres de colisión
        try:
            result = self._infer(object_xyz, scene_xyz)
        except (requests.exceptions.RequestException, ValueError) as exc:
            return self._fail(response, f"error llamando a graspgen: {exc}")

        if not result.get("success"):
            return self._fail(
                response, f"graspgen: {result.get('error', 'error desconocido')}"
            )
        grasps = result.get("grasps", [])
        if not grasps:
            return self._fail(
                response, "sin_agarre_alcanzable: graspgen no devolvió agarres"
            )

        # grasps come in the frame of the object cloud, already sorted by score
        header = request.object_cloud.header
        for grasp in grasps:
            # float(): JSON may send integers (e.g. a quaternion [0, 0, 0, 1]), and an int in
            # a float field aborts the whole node when the response is serialized
            x, y, z = (float(v) for v in grasp["position"])
            qx, qy, qz, qw = (float(v) for v in grasp["quaternion_xyzw"])
            pose = PoseStamped(header=header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = qx, qy, qz, qw
            response.top_grasps.append(pose)
        response.best_pose = response.top_grasps[0]

        self.top_grasps_pub.publish(
            PoseArray(header=header, poses=[p.pose for p in response.top_grasps])
        )
        self.best_grasp_pub.publish(response.best_pose)

        response.success = True
        # .get: the stats are only informative, a missing one must not break the response
        response.message = (
            f"score={grasps[0].get('score', float('nan')):.3f}, top={result.get('count', len(grasps))}, "
            f"libres={result.get('collision_free', '?')}/{result.get('total_candidates', '?')}, "
            f"dt={result.get('dt', '?')}s"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspgenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
