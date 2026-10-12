#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

GROUP_NAME = "right_arm"
TIP_LINK = "TCP"

# Configuración articular de prueba (en radianes)
# [1.5, -0.5, 0.0, -1.5, 0.3, 0.2] (~86°, -28°, 0°, -86°, 17°, 11°)
TEST_JOINTS = [1.5, -0.5, 0.0, -1.5, 0.3, 0.2]


class FKPosePublisher(Node):
  def __init__(self):
    super().__init__("fk_pose_publisher")

    # Cargar configuración de MoveIt
    moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()
    config_dict = moveit_config.to_dict()
    config_dict["planning_pipelines"] = {"pipeline_names": config_dict["planning_pipelines"]}
    config_dict["use_sim_time"] = False

    self.moveit = MoveItPy(node_name="fk_calculator_py", config_dict=config_dict, provide_planning_service=False)
    self.psm = self.moveit.get_planning_scene_monitor()
    self.pub = self.create_publisher(PoseStamped, "/grasp_candidate_pose", 10)

    self.get_logger().info("Calculando Cinemática Directa (FK)...")
    self.publish_fk_pose()

  def publish_fk_pose(self):
    with self.psm.read_only() as scene:
      robot_state = scene.current_state

    # Asignar posiciones articulares al grupo
    robot_state.set_joint_group_positions(GROUP_NAME, TEST_JOINTS)
    robot_state.update()

    # Obtener la transformación global del TIP_LINK (pose cartesiana)
    pose_eigen = robot_state.get_global_link_transform(TIP_LINK)

    # Construir PoseStamped
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    msg.header.stamp = self.get_clock().now().to_msg()

    # Posición
    msg.pose.position.x = float(pose_eigen[0, 3])
    msg.pose.position.y = float(pose_eigen[1, 3])
    msg.pose.position.z = float(pose_eigen[2, 3])

    # Orientación (matriz de rotación a cuaternión)
    from scipy.spatial.transform import Rotation as R
    rot_matrix = pose_eigen[:3, :3]
    q = R.from_matrix(rot_matrix).as_quat() # [x, y, z, w]

    msg.pose.orientation.x = float(q[0])
    msg.pose.orientation.y = float(q[1])
    msg.pose.orientation.z = float(q[2])
    msg.pose.orientation.w = float(q[3])

    # Imprimir en consola la pose generada
    self.get_logger().info("\n--- Pose Cartesiana Calculada (FK) ---")
    self.get_logger().info(f"Position:  x={msg.pose.position.x:.4f}, y={msg.pose.position.y:.4f}, z={msg.pose.position.z:.4f}")
    self.get_logger().info(f"Orientation: x={msg.pose.orientation.x:.4f}, y={msg.pose.orientation.y:.4f}, z={msg.pose.orientation.z:.4f}, w={msg.pose.orientation.w:.4f}")
    self.get_logger().info(
      f'\nComando equivalente ros2 topic pub:\n'
      f'ros2 topic pub -1 /grasp_candidate_pose geometry_msgs/msg/PoseStamped "{{\n'
      f' header: {{frame_id: \'base_link\'}},\n'
      f' pose: {{\n'
      f'  position: {{x: {msg.pose.position.x:.4f}, y: {msg.pose.position.y:.4f}, z: {msg.pose.position.z:.4f}}},\n'
      f'  orientation: {{x: {msg.pose.orientation.x:.4f}, y: {msg.pose.orientation.y:.4f}, z: {msg.pose.orientation.z:.4f}, w: {msg.pose.orientation.w:.4f}}}\n'
      f' }}\n'
      f'}}"'
    )

    # Publicar mensaje
    self.pub.publish(msg)
    self.get_logger().info("Pose publicada exitosamente en /grasp_candidate_pose")


def main():
  rclpy.init()
  node = FKPosePublisher()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
