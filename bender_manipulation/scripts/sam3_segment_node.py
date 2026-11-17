#!/usr/bin/env python3
"""Nodo de segmentación SAM3: de nube+prompt a nube de puntos del objeto.

sam3 no es un nodo ROS2, es un servidor Flask aparte (http://localhost:5001),
por eso este nodo le habla por HTTP (requests) en vez de un cliente de
servicio ROS2. El pipeline, disparado por el servicio 'segment_scene'
(uchile_srvs/Segmentation), sigue estos pasos:

  Inferencia(imagen, prompt):
      POST /segment/all a sam3 -> máscaras [u, v] (una por objeto) + scores.
      La imagen es el último frame de color recibido; el prompt viene en la
      request (si viene vacío se usa el parámetro 'prompt').

  Selección:
      Se toma la máscara de mayor score (sam3 ya las entrega ordenadas
      descendente). Si ese score < min_mask_score, falla la inferencia.

  Proyección:
      Cada punto de cloud_in se lleva al frame de la cámara de color (TF) y
      se proyecta a la imagen con los intrínsecos de camera_info.

  Objeto = puntos cuyo píxel cae dentro de la máscara
  Escena = todos los demás puntos de cloud_in (incluye los que quedan fuera
      de la imagen de color)

  Nubes (en el mismo frame y stamp de cloud_in):
      object_segment = puntos de Objeto
      scene_segment  = puntos de Escena (con subsampling opcional)

  Validación:
      Si object_segment tiene menos de min_object_points, falla la inferencia.
"""
import base64

import cv2
import numpy as np
import requests
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from uchile_srvs.srv import Segmentation


class Sam3SegmentNode(Node):
    def __init__(self):
        super().__init__('sam3_segment_node')

        self.declare_parameter('sam3_url', 'http://tesistas-MS-7E12.local:5001')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('prompt', 'soda can')
        self.declare_parameter('min_mask_score', 0.20)
        self.declare_parameter('min_object_points', 50)
        self.declare_parameter('request_timeout', 15.0)

        # Parámetro de subsampling para la escena (1 = sin subsampling, 2 = toma 1 de cada 2 puntos, etc.)
        self.declare_parameter('scene_subsample_factor', 100)

        self.bridge = CvBridge()
        self.last_color = None
        self.last_camera_info = None

        # the listener spins in its own thread, so TF keeps arriving while the service callback runs
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(Image, self.get_parameter('color_topic').value, self._color_cb, 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self._camera_info_cb, 10)

        # same clouds as the service response, published for RViz
        self.object_cloud_pub = self.create_publisher(PointCloud2, '/sam3/object_cloud', 10)
        self.scene_cloud_pub = self.create_publisher(PointCloud2, '/sam3/scene_cloud', 10)
        # color frame sent to sam3 with the selected mask drawn on top
        self.debug_image_pub = self.create_publisher(Image, '/sam3/debug_image', 10)
        self.create_service(Segmentation, 'segment_scene', self._segment_scene_cb)

        self.get_logger().info(
            f"sam3_segment_node listo. servicio 'segment_scene' -> "
            f"{self.get_parameter('sam3_url').value}/segment/all"
        )

    def _color_cb(self, msg: Image):
        self.last_color = msg

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
    # Objeto = puntos dentro de la máscara ; Escena = el resto de la nube
    # ------------------------------------------------------------------
    def _to_camera_frame(self, points: np.ndarray, cloud_frame: str, camera_frame: str):
        """Points (Nx3) expressed in the color camera frame, or None if TF is not available."""
        if cloud_frame == camera_frame:
            return points
        try:
            # latest transform: the robot is expected to be still while it segments
            tf = self.tf_buffer.lookup_transform(
                camera_frame, cloud_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except tf2_ros.TransformException as exc:
            self.get_logger().error(f"No transform from '{cloud_frame}' to '{camera_frame}': {exc}")
            return None

        q, t = tf.transform.rotation, tf.transform.translation
        rotation = np.array([
            [1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y)],
            [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.w * q.x)],
            [2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)],
        ])
        return points @ rotation.T + np.array([t.x, t.y, t.z])

    @staticmethod
    def _project(points_cam: np.ndarray, K: np.ndarray, shape):
        """Pixel (u, v) of every 3D point via the pinhole model with the camera_info intrinsics K,
        plus a flag for the points that land inside the image."""
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        u = np.zeros(points_cam.shape[0], dtype=np.int64)
        v = np.zeros(points_cam.shape[0], dtype=np.int64)
        # points behind the camera have no pixel
        in_front = points_cam[:, 2] > 0.0
        x, y, z = points_cam[in_front].T
        u[in_front] = np.round(fx * x / z + cx).astype(np.int64)
        v[in_front] = np.round(fy * y / z + cy).astype(np.int64)
        visible = in_front & (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
        return u, v, visible

    @staticmethod
    def _build_cloud(points: np.ndarray, header: Header) -> PointCloud2:
        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def _publish_debug_image(self, color_bgr: np.ndarray, mask, text: str, accepted: bool):
        """Publishes the frame with the mask filled and outlined, green if it was accepted
        and red if it was rejected, plus a text line with the result."""
        image = color_bgr.copy()
        color = (0, 200, 0) if accepted else (0, 0, 220)
        if mask is not None:
            overlay = image.copy()
            overlay[mask] = color
            image = cv2.addWeighted(overlay, 0.4, image, 0.6, 0.0)
            contours, _ = cv2.findContours(
                mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, color, 2)
        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header = self.last_color.header
        self.debug_image_pub.publish(msg)

    @staticmethod
    def _fail(response: Segmentation.Response, message: str):
        # the clouds of the response stay empty
        response.success = False
        response.message = message
        return response

    # ------------------------------------------------------------------
    def _segment_scene_cb(self, request: Segmentation.Request, response: Segmentation.Response):
        if self.last_color is None or self.last_camera_info is None:
            return self._fail(response, 'faltan datos de cámara (color o camera_info)')

        cloud_in = request.cloud_in
        if cloud_in.width * cloud_in.height == 0:
            return self._fail(response, 'cloud_in is empty')
        try:
            points = point_cloud2.read_points_numpy(cloud_in, field_names=('x', 'y', 'z'), skip_nans=True)
        except (AssertionError, ValueError, KeyError) as exc:
            return self._fail(response, f'cloud_in has no usable x, y, z fields: {exc}')
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)

        color_bgr = self.bridge.imgmsg_to_cv2(self.last_color, desired_encoding='bgr8')
        K = np.array(self.last_camera_info.k)
        camera_frame = self.last_camera_info.header.frame_id

        points_cam = self._to_camera_frame(points, cloud_in.header.frame_id, camera_frame)
        if points_cam is None:
            return self._fail(
                response, f"no TF from '{cloud_in.header.frame_id}' to '{camera_frame}'")

        # an empty prompt in the request falls back to the node parameter
        prompt = request.prompt or self.get_parameter('prompt').value
        min_mask_score = self.get_parameter('min_mask_score').value
        min_object_points = self.get_parameter('min_object_points').value
        scene_subsample_factor = self.get_parameter('scene_subsample_factor').value

        # Inferencia(imagen, prompt) -> máscaras + scores
        try:
            objects = self._infer(color_bgr, prompt)
        except (requests.exceptions.RequestException, ValueError) as exc:
            return self._fail(response, f'error llamando a sam3: {exc}')

        if not objects:
            self._publish_debug_image(color_bgr, None, f"'{prompt}': no detections", False)
            return self._fail(
                response, f'sin_agarre_alcanzable: sam3 no detectó objetos de clase {prompt}')

        # Selecciona máscara con score más alto (sam3 ya las entrega ordenadas)
        best = objects[0]
        mask = self._decode_mask(best['mask_png'], color_bgr.shape[:2])
        # drawn before the checks, so a rejected mask can be seen too
        accepted = best['score'] >= min_mask_score
        self._publish_debug_image(
            color_bgr, mask, f"'{prompt}' score={best['score']:.3f}", accepted)
        if not accepted:
            return self._fail(
                response,
                f"score {best['score']:.3f} bajo min_mask_score={min_mask_score}, {prompt = }")

        # Objeto = puntos cuyo píxel cae en la máscara ; Escena = todos los demás
        u, v, visible = self._project(points_cam, K, mask.shape)
        in_mask = np.zeros(points.shape[0], dtype=bool)
        in_mask[visible] = mask[v[visible], u[visible]]

        # the output keeps the coordinates of cloud_in, the camera frame is only used to project
        object_points = points[in_mask]
        scene_points = points[~in_mask]

        # Validación: object_cloud con menos de min_object_points -> falla
        if object_points.shape[0] < min_object_points:
            return self._fail(
                response,
                f"sin_agarre_alcanzable: object_cloud tiene {object_points.shape[0]} puntos "
                f"(< {min_object_points})")

        # Subsampling de la nube de puntos de la escena
        if scene_subsample_factor > 1:
            scene_points = scene_points[::scene_subsample_factor]

        # same frame and stamp as cloud_in, so the segments match the capture they come from
        response.object_segment = self._build_cloud(object_points, cloud_in.header)
        response.scene_segment = self._build_cloud(scene_points, cloud_in.header)
        self.object_cloud_pub.publish(response.object_segment)
        self.scene_cloud_pub.publish(response.scene_segment)

        response.success = True
        response.message = (
            f"score={best['score']:.3f} object_cloud={object_points.shape[0]}pts "
            f"scene_cloud={scene_points.shape[0]}pts (subsample_factor={scene_subsample_factor})"
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
