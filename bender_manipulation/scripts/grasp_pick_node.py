#!/usr/bin/env python3
"""
Nodo de recogida (pick) para el brazo derecho de Bender.

Implementa dos algoritmos:

  Algoritmo 2 - Selección de la pose de agarre alcanzable:
      Recorre las poses de agarre candidatas (ordenadas por confianza) y
      devuelve la primera cuya pose de contacto y su pose de aproximación
      (retrocedida una distancia d sobre el eje de aproximación) sean
      alcanzables por IK y estén libres de colisión contra la escena de
      planificación (incluye el octomap M cuando está disponible).

  Algoritmo 3 - Ejecución de un tramo de movimiento:
      Planifica entre una configuración de inicio y una configuración
      objetivo con el pipeline de planificación configurado, y ejecuta la
      trayectoria resultante si la planificación tuvo éxito.

Modo de prueba manual (aún no hay nodo de detección de agarres): publica una
geometry_msgs/PoseStamped en /grasp_candidate_pose y el nodo la trata como
única candidata (G = [g]) para ir probando el pipeline extremo a extremo.

    ros2 topic pub -1 /grasp_candidate_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {position: {x: 0.5, y: -0.2, z: 0.4}, orientation: {w: 1.0}}}"
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder


GROUP_NAME = "right_arm"
TIP_LINK = "TCP"


def _retroceder(pose: Pose, distance: float) -> Pose:
    """gpre = retroceder(g, d)

    Retrocede 'distance' metros a lo largo del eje de aproximación (eje Z
    local de la orientación del agarre), manteniendo la misma orientación.
    """
    qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

    # Eje Z local de la pose, expresado en el frame de referencia: R(q) @ [0, 0, 1]
    zx = 2.0 * (qx * qz + qw * qy)
    zy = 2.0 * (qy * qz - qw * qx)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)

    pre = Pose()
    pre.orientation = pose.orientation
    pre.position.x = pose.position.x - distance * zx
    pre.position.y = pose.position.y - distance * zy
    pre.position.z = pose.position.z - distance * zz
    return pre


class GraspPickNode(Node):
    def __init__(self):
        super().__init__("grasp_pick_node")

        self.declare_parameter("approach_distance", 0.10)  # d, en metros
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("ik_random_restarts", 5)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("planning_attempts", 10)
        # Tolerancias del goal en el tip (TCP) al planificar el tramo. La posición
        # se mantiene ceñida (el agarre debe caer donde se pidió); la orientación
        # se deja holgada porque el goal por defecto de MoveIt usa una tolerancia
        # angular ~0 (epsilon de máquina) que hace que el planner batalle para
        # converger, incluso cuando la pose sí es alcanzable.
        self.declare_parameter("goal_position_tolerance", 0.005)     # metros
        self.declare_parameter("goal_orientation_tolerance", 0.35)   # radianes (~20°)
        # Cuánto puede diferir el primer punto de la trayectoria del estado real
        # del robot al momento de ejecutar (asentamiento del controlador entre
        # tramos). Con el valor por defecto (0.01 rad) trajectory_execution_manager
        # rechazaba tramos válidos con "Invalid Trajectory: start point deviates...".
        self.declare_parameter("allowed_start_tolerance", 0.1)       # radianes
        # Pausa tras ejecutar un tramo para que el robot termine de asentarse
        # físicamente antes de que el siguiente tramo capture el estado actual
        # como punto de partida (evita "start point deviates..." si el robot
        # sigue frenando/oscilando levemente cuando arranca la planificación).
        self.declare_parameter("settle_time_after_segment", 0.3)     # segundos

        moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()
        config_dict = moveit_config.to_dict()

        # MoveItCpp (usado por MoveItPy) espera 'planning_pipelines.pipeline_names'
        # (dict anidado), mientras que MoveItConfigsBuilder entrega 'planning_pipelines'
        # como lista plana (formato que sí entiende el nodo move_group clásico).
        # Sin este ajuste, MoveItPy falla con "Failed to load planning pipelines".
        config_dict["planning_pipelines"] = {"pipeline_names": config_dict["planning_pipelines"]}

        te = config_dict.get("trajectory_execution", {})
        te["allowed_start_tolerance"] = self.get_parameter("allowed_start_tolerance").value
        config_dict["trajectory_execution"] = te

        # Necesario para sincronizar con el reloj simulado de Gazebo; sin esto
        # trajectory_execution_manager rechaza el estado articular por "obsoleto"
        # y nunca llega a mover el robot de verdad.
        # (Requiere el parche en moveit_ros_planning/trajectory_execution_manager
        # que quita el rechazo de parámetros desconocidos como qos_overrides.*;
        # ver PR moveit2#3689 / issue moveit2#2940 - de lo contrario esto crashea
        # con rclcpp::exceptions::InvalidParameterValueException.)
        config_dict["use_sim_time"] = False

        # provide_planning_service=False: ya hay un move_group corriendo (para RViz);
        # este nodo no debe competir por los mismos nombres de servicio/acción.
        self.moveit = MoveItPy(
            node_name="grasp_pick_moveit_py",
            config_dict=config_dict,
            provide_planning_service=False,
        )
        self.arm = self.moveit.get_planning_component(GROUP_NAME)
        self.psm = self.moveit.get_planning_scene_monitor()

        self.status_pub = self.create_publisher(String, "/grasp_pick_status", 10)
        self.create_subscription(PoseStamped, "/grasp_candidate_pose", self.grasp_candidate_cb, 10)

        self.get_logger().info(
            f"grasp_pick_node listo (grupo='{GROUP_NAME}', tip='{TIP_LINK}'). "
            "Publica geometry_msgs/PoseStamped en /grasp_candidate_pose para probar."
        )

    # ------------------------------------------------------------------
    # Algoritmo 2: Selección de la pose de agarre alcanzable
    # ------------------------------------------------------------------
    def select_reachable_grasp(self, candidates: list, d: float):
        """
        Entrada: candidates (G) ordenadas por confianza (mayor primero), d.
        Salida: (g, gpre, qg, qpre) del primer candidato alcanzable, o None
                si ningún candidato admite solución ("sin_agarre_alcanzable").
        """
        for g in candidates:  # recorrido en orden decreciente de confianza
            qg = self._compute_ik(g)                      # qg = IK(g)
            gpre = self._offset_stamped(g, d)              # gpre = retroceder(g, d)
            qpre = self._compute_ik(gpre)                  # qpre = IK(gpre)

            qg_valid = qg is not None and self._alcanzable(qg)
            qpre_valid = qpre is not None and self._alcanzable(qpre)
            self.get_logger().info(
                f"Candidato: IK(g)={'ok' if qg is not None else 'fail'} "
                f"alcanzable(g)={qg_valid} IK(gpre)={'ok' if qpre is not None else 'fail'} "
                f"alcanzable(gpre)={qpre_valid}"
            )

            if qg_valid and qpre_valid:
                return g, gpre, qg, qpre                   # primer candidato alcanzable

        return None  # "sin_agarre_alcanzable"

    def _offset_stamped(self, g: PoseStamped, d: float) -> PoseStamped:
        gpre = PoseStamped()
        gpre.header = g.header
        gpre.pose = _retroceder(g.pose, d)
        return gpre

    def _compute_ik(self, pose_stamped: PoseStamped):
        """qg = IK(g): configuración que alcanza la pose de contacto, o None si no hay solución.

        El solver KDL (Newton-Raphson) es local: si la semilla inicial (estado
        actual del robot) está cerca de una singularidad cinemática, puede
        oscilar sin converger aunque exista solución. Por eso, si la búsqueda
        desde el estado actual falla, se reintenta unas pocas veces desde
        configuraciones aleatorias dentro de los límites de las articulaciones.
        """
        timeout = self.get_parameter("ik_timeout").value

        with self.psm.read_only() as scene:
            base_model = scene.current_state.robot_model
            current_joint_positions = scene.current_state.joint_positions

        # Intento 1: semilla = estado actual del robot.
        state = RobotState(base_model)
        state.joint_positions = current_joint_positions
        state.update()
        if state.set_from_ik(GROUP_NAME, pose_stamped.pose, TIP_LINK, timeout):
            state.update()
            return state

        # Intentos adicionales: semillas aleatorias (reinicio para escapar de singularidades).
        jmg = state.robot_model.get_joint_model_group(GROUP_NAME)
        for _ in range(self.get_parameter("ik_random_restarts").value):
            state = RobotState(base_model)
            state.set_to_random_positions(jmg)
            state.update()
            if state.set_from_ik(GROUP_NAME, pose_stamped.pose, TIP_LINK, timeout):
                state.update()
                return state

        return None

    def _alcanzable(self, state: RobotState) -> bool:
        """alcanzable(q, M): sin colisión contra la escena de planificación (incluye el octomap M)."""
        with self.psm.read_only() as scene:
            return scene.is_state_valid(state, GROUP_NAME)

    def _goal_constraints_with_slack(self, goal_pose: PoseStamped) -> Constraints:
        """Constraints de posición+orientación para TIP_LINK, con tolerancia de
        posición ceñida y de orientación holgada (ver parámetros del nodo)."""
        position_tolerance = self.get_parameter("goal_position_tolerance").value
        orientation_tolerance = self.get_parameter("goal_orientation_tolerance").value

        pos = PositionConstraint()
        pos.header = goal_pose.header
        pos.link_name = TIP_LINK
        pos.target_point_offset.x = 0.0
        pos.target_point_offset.y = 0.0
        pos.target_point_offset.z = 0.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tolerance])
        pos.constraint_region.primitives.append(sphere)
        pos.constraint_region.primitive_poses.append(goal_pose.pose)
        pos.weight = 1.0

        orient = OrientationConstraint()
        orient.header = goal_pose.header
        orient.link_name = TIP_LINK
        orient.orientation = goal_pose.pose.orientation
        orient.absolute_x_axis_tolerance = orientation_tolerance
        orient.absolute_y_axis_tolerance = orientation_tolerance
        orient.absolute_z_axis_tolerance = orientation_tolerance
        orient.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(orient)
        return constraints

    # ------------------------------------------------------------------
    # Algoritmo 3: Ejecución de un tramo de movimiento
    # ------------------------------------------------------------------
    def execute_segment(self, goal_pose: PoseStamped) -> str:
        """
        tau = P(qstart, qgoal, M)
        Si no hay trayectoria -> "fallo_planificacion" (se reporta al orquestador).
        Si la hay -> ejecutar(tau) -> "exito".

        qstart se toma como el estado ACTUAL del robot justo antes de planificar
        (no un RobotState capturado antes, ver más abajo por qué) y qgoal se
        expresa como pose cartesiana en TIP_LINK, con tolerancia de posición
        ceñida y de orientación holgada, en vez del goal articular por defecto
        (tolerancia ~0 por articulación).
        """
        # set_start_state_to_current_state() lee el estado en vivo justo antes
        # de planificar. Usar un RobotState capturado antes (p.ej. al comienzo
        # del callback) deja una ventana en la que el robot real se sigue
        # asentando/moviendo un poco; al ejecutar, trajectory_execution_manager
        # compara el primer punto del plan contra el estado real en ese momento
        # y lo rechaza si difiere más de allowed_start_tolerance (~0.01 rad):
        # "Invalid Trajectory: start point deviates from current robot state".
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(motion_plan_constraints=[self._goal_constraints_with_slack(goal_pose)])

        params = PlanRequestParameters(self.moveit, "")
        params.planning_pipeline = "ompl"
        params.planning_time = self.get_parameter("planning_time").value
        params.planning_attempts = self.get_parameter("planning_attempts").value

        plan_result = self.arm.plan(single_plan_parameters=params)  # tau = P(qstart, qgoal, M)

        if not plan_result:  # tau == vacio
            return "fallo_planificacion"

        exec_status = self.moveit.execute(plan_result.trajectory, controllers=[])  # ejecutar(tau)
        if not exec_status:
            self.get_logger().error(f"Ejecución rechazada/fallida: {exec_status.status}")
            return "fallo_ejecucion"

        settle_time = self.get_parameter("settle_time_after_segment").value
        if settle_time > 0.0:
            time.sleep(settle_time)

        return "exito"

    # ------------------------------------------------------------------
    def grasp_candidate_cb(self, msg: PoseStamped):
        d = self.get_parameter("approach_distance").value
        self.get_logger().info(f"Candidato de agarre recibido (frame={msg.header.frame_id}), d={d:.3f} m")

        # Por ahora G = [msg] (una sola pose publicada a mano). Cuando exista
        # un nodo de detección de agarres, este callback puede acumular una
        # lista de PoseStamped ordenada por confianza y pasarla completa aquí.
        result = self.select_reachable_grasp([msg], d)

        if result is None:
            self.get_logger().warn("sin_agarre_alcanzable")
            self.status_pub.publish(String(data="sin_agarre_alcanzable"))
            return

        g, gpre, _qg, _qpre = result
        self.get_logger().info("Agarre alcanzable encontrado, ejecutando aproximación...")

        # Tramo 1: configuración actual -> pose de aproximación (pre-agarre)
        status = self.execute_segment(gpre)
        if status != "exito":
            self.get_logger().error(f"Aproximación: {status}")
            self.status_pub.publish(String(data=status))
            return

        # Tramo 2: pose de aproximación -> pose de agarre final
        status = self.execute_segment(g)
        self.get_logger().info(f"Agarre: {status}")
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = GraspPickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
