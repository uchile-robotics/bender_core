#!/usr/bin/env python3
"""Nodo de segmentación SAM3: de imagen+prompt a nube de puntos del objeto.

sam3 no es un nodo ROS2, es un servidor Flask aparte (http://localhost:5001),
por eso este nodo le habla por HTTP (requests) en vez de un cliente de
servicio ROS2. El pipeline, disparado por el servicio 'segment_scene'
(std_srvs/Trigger), sigue estos pasos:

  Inferencia(imagen, prompt):
      POST /segment/all a sam3 -> máscaras [u, v] (una por objeto) + scores.

  Selección:
      Se toma la máscara de mayor score (sam3 ya las entrega ordenadas
      descendente). Si ese score < min_mask_score, falla la inferencia.

  Objeto = máscara AND profundidad_valida
  Escena = NOT máscara AND profundidad_valida   (exclusion_mascara = la
      máscara del objeto seleccionado: la escena es todo lo demás con
      profundidad válida)

  Nubes:
      object_cloud = deproyección 3D de los píxeles de Objeto
      scene_cloud  = deproyección 3D de los píxeles de Escena

  Validación:
      Si object_cloud tiene menos de min_object_points, falla la inferencia.
"""
import base64
import io

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class Sam3SegmentNode(Node):
    def __init__(self):
        super().__init__('sam3_segment_node')

        self.declare_parameter('sam3_url', 'http://192.168.1.136:5001')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('prompt', 'soda can')
        self.declare_parameter('min_mask_score', 0.20)
        self.declare_parameter('min_object_points', 50)
        self.declare_parameter('request_timeout', 15.0)

        self.bridge = CvBridge()
        self.last_color = None
        self.last_depth = None
        self.last_camera_info = None

        self.create_subscription(Image, self.get_parameter('color_topic').value, self._color_cb, 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self._depth_cb, 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self._camera_info_cb, 10)

        self.object_cloud_pub = self.create_publisher(PointCloud2, '/sam3/object_cloud', 10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, '/sam3/scene_cloud', 10)
        self.create_service(Trigger, 'segment_scene', self._segment_scene_cb)

        self.get_logger().info(
            f"sam3_segment_node listo. servicio 'segment_scene' -> "
            f"{self.get_parameter('sam3_url').value}/segment/all"
        )

    def _color_cb(self, msg: Image):
        self.last_color = msg

    def _depth_cb(self, msg: Image):
        self.last_depth = msg

    def _camera_info_cb(self, msg: CameraInfo):
        self.last_camera_info = msg

    # ------------------------------------------------------------------
    # Inferencia(imagen, prompt) -> máscaras + scores
    # ------------------------------------------------------------------
    def _infer(self, color_bgr: np.ndarray, prompt: str):
        """Llama a sam3 /segment/all. Devuelve lista de dicts (ya viene
        ordenada por score descendente) con al menos 'score' y 'mask_png',
        o levanta requests.exceptions.RequestException si falla la llamada."""
        ok, png = cv2.imencode('.png', color_bgr)
        if not ok:
            raise ValueError('no se pudo codificar el frame a PNG')

        sam3_url = self.get_parameter('sam3_url').value
        timeout = self.get_parameter('request_timeout').value

        reply = requests.post(
            f'{sam3_url}/segment/all',
            files={'image': ('frame.png', png.tobytes(), 'image/png')},
            data={'text': prompt},
            timeout=timeout,
        )
        reply.raise_for_status()
        return reply.json()['objects']

    @staticmethod
    def _decode_mask(mask_png_b64: str, shape) -> np.ndarray:
        """PNG binaria en base64 -> máscara bool [H, W]."""
        raw = base64.b64decode(mask_png_b64)
        arr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        if arr.shape != shape:
            arr = cv2.resize(arr, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
        return arr > 127

    # ------------------------------------------------------------------
    # Objeto = máscara AND profundidad ; Escena = NOT máscara AND profundidad
    # ------------------------------------------------------------------
    @staticmethod
    def _deproject(pixel_mask: np.ndarray, depth_m: np.ndarray, K: np.ndarray) -> np.ndarray:
        """Puntos 3D (Nx3, metros) de los píxeles marcados en pixel_mask,
        vía el modelo pinhole con los intrínsecos K de camera_info."""
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        ys, xs = np.where(pixel_mask)
        z = depth_m[ys, xs]
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        return np.stack([x, y, z], axis=-1).astype(np.float32)

    def _build_cloud(self, points: np.ndarray, frame_id: str) -> PointCloud2:
        header = self.get_clock().now().to_msg()
        from std_msgs.msg import Header
        h = Header(frame_id=frame_id, stamp=header)
        return point_cloud2.create_cloud_xyz32(h, points.tolist())

    # ------------------------------------------------------------------
    def _segment_scene_cb(self, request: Trigger.Request, response: Trigger.Response):
        if self.last_color is None or self.last_depth is None or self.last_camera_info is None:
            response.success = False
            response.message = f'faltan datos de cámara (color{self.last_color} \n/depth{self.last_depth}\n/camera_info{self.last_camera_info})'
            return response

        color_bgr = self.bridge.imgmsg_to_cv2(self.last_color, desired_encoding='bgr8')
        depth_raw = self.bridge.imgmsg_to_cv2(self.last_depth, desired_encoding='passthrough')
        depth_m = depth_raw.astype(np.float32) / 1000.0  # realsense entrega 16UC1 en mm
        K = np.array(self.last_camera_info.k)

        prompt = self.get_parameter('prompt').value
        min_mask_score = self.get_parameter('min_mask_score').value
        min_object_points = self.get_parameter('min_object_points').value

        # Inferencia(imagen, prompt) -> máscaras + scores
        try:
            objects = self._infer(color_bgr, prompt)
        except (requests.exceptions.RequestException, ValueError) as exc:
            response.success = False
            response.message = f'error llamando a sam3: {exc}'
            return response

        if not objects:
            response.success = False
            response.message = f'sin_agarre_alcanzable: sam3 no detectó objetos de clase {prompt}'
            return response

        # Selecciona máscara con score más alto (sam3 ya las entrega ordenadas)
        best = objects[0]
        if best['score'] < min_mask_score:
            response.success = False
            response.message = (
                f"score {best['score']:.3f} bajo min_mask_score={min_mask_score}, {prompt = }"
            )
            return response

        mask = self._decode_mask(best['mask_png'], depth_m.shape)
        depth_valid = np.isfinite(depth_m) & (depth_m > 0.0)

        # Objeto = máscara AND profundidad ; exclusion_mascara = máscara del objeto
        object_px = mask & depth_valid
        scene_px = (~mask) & depth_valid

        frame_id = self.last_depth.header.frame_id
        object_points = self._deproject(object_px, depth_m, K)
        scene_points = self._deproject(scene_px, depth_m, K)

        # Validación: object_cloud con menos de min_object_points -> falla
        if object_points.shape[0] < min_object_points:
            response.success = False
            response.message = (
                f"sin_agarre_alcanzable: object_cloud tiene {object_points.shape[0]} puntos "
                f"(< {min_object_points})"
            )
            return response

        self.object_cloud_pub.publish(self._build_cloud(object_points, frame_id))
        self.scene_cloud_pub.publish(self._build_cloud(scene_points, frame_id))

        response.success = True
        response.message = (
            f"score={best['score']:.3f} object_cloud={object_points.shape[0]}pts "
            f"scene_cloud={scene_points.shape[0]}pts"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Sam3SegmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
