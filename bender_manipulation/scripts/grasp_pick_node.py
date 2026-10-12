#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

MOVEIT_ERROR_MAP = {
    MoveItErrorCodes.SUCCESS: "Éxito",
    MoveItErrorCodes.FAILURE: "Fallo no especificado",
    MoveItErrorCodes.PLANNING_FAILED: "Fallo general de planificación",
    MoveItErrorCodes.TIMED_OUT: "Tiempo límite agotado (Timeout)",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "Estado inicial en colisión",
    MoveItErrorCodes.GOAL_IN_COLLISION: "Estado objetivo en colisión",
    MoveItErrorCodes.NO_IK_SOLUTION: "Sin solución de cinemática inversa (IK)",
}

GROUP_NAME = "right_arm"
TIP_LINK = "TCP"


def get_error_string(code: int) -> str:
    return MOVEIT_ERROR_MAP.get(code, f"Código de error {code}")


# ---------------------------------------------------------------- quaterniones

def _q_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        # Cuaternión nulo (mensaje sin inicializar). Devolvemos identidad.
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_from_axis_angle(axis, angle):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax / n, ay / n, az / n
    s = math.sin(angle / 2.0)
    return (ax * s, ay * s, az * s, math.cos(angle / 2.0))


def _axis_vector(q, axis: str):
    """Devuelve el eje X, Y o Z de la pose expresado en el frame padre."""
    qx, qy, qz, qw = q
    if axis == "x":
        return (1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy + qw * qz),
                2.0 * (qx * qz - qw * qy))
    if axis == "y":
        return (2.0 * (qx * qy - qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz + qw * qx))
    return (2.0 * (qx * qz + qw * qy),
            2.0 * (qy * qz - qw * qx),
            1.0 - 2.0 * (qx * qx + qy * qy))


def _retroceder(pose: Pose, distance: float, approach_axis: str = "z") -> Pose:
    """Retrocede `distance` metros a lo largo del eje de aproximación del TCP."""
    q = _q_normalize((pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w))
    ax, ay, az = _axis_vector(q, approach_axis)

    pre = Pose()
    pre.orientation.x, pre.orientation.y, pre.orientation.z, pre.orientation.w = q
    pre.position.x = pose.position.x - distance * ax
    pre.position.y = pose.position.y - distance * ay
    pre.position.z = pose.position.z - distance * az
    return pre


def _aplicar_bias(pose: Pose, bias: float, approach_axis: str = "z") -> Pose:
    """Avanza `bias` metros a lo largo del eje de aproximación del TCP."""
    return _retroceder(pose, -bias, approach_axis)


def _variantes_simetria(pose_stamped: PoseStamped, approach_axis: str, angulos):
    """Genera poses equivalentes girando el gripper sobre su eje de aproximación."""
    eje = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}[approach_axis]
    q0 = _q_normalize((pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                       pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w))
    salida = []
    for ang in angulos:
        qx, qy, qz, qw = _q_mul(q0, _q_from_axis_angle(eje, ang))
        p = PoseStamped()
        p.header = pose_stamped.header
        p.pose.position = pose_stamped.pose.position
        p.pose.orientation.x, p.pose.orientation.y = qx, qy
        p.pose.orientation.z, p.pose.orientation.w = qz, qw
        salida.append((math.degrees(ang), p))
    return salida


# --------------------------------------------------------------------- el nodo

class GraspPickNode(Node):
    def __init__(self):
        super().__init__("grasp_pick_node")
        self.cb_group = ReentrantCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.declare_parameter("approach_distance", 0.10)
        self.declare_parameter("approach_axis", "z")          # eje del TCP que apunta al objeto
        self.declare_parameter("symmetry_angles_deg", [0.0, 180.0, 90.0, -90.0])
        self.declare_parameter("ik_timeout", 0.2)
        self.declare_parameter("ik_random_restarts", 25)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 5)
        self.declare_parameter("allowed_start_tolerance", 0.1)
        self.declare_parameter("settle_time_after_segment", 0.3)
        self.declare_parameter("bias", 0.10)                  # Desplazamiento adicional hacia adelante en metros
        self.declare_parameter("skip_collision_check_at_grasp", False)
        self.declare_parameter("cartesian_max_step", 0.005)   # resolución de la interpolación en metros
        self.declare_parameter("cartesian_min_fraction", 0.99)
        self.declare_parameter("cartesian_avoid_collisions", True)
        self.declare_parameter("cartesian_timeout", 10.0)

        moveit_config = MoveItConfigsBuilder("bender", package_name="bender_moveit_config").to_moveit_configs()
        config_dict = moveit_config.to_dict()
        config_dict["planning_pipelines"] = {"pipeline_names": config_dict["planning_pipelines"]}

        te = config_dict.get("trajectory_execution", {})
        te["allowed_start_tolerance"] = self.get_parameter("allowed_start_tolerance").value
        config_dict["trajectory_execution"] = te
        config_dict["use_sim_time"] = False

        self.moveit = MoveItPy(
            node_name="grasp_pick_moveit_py",
            config_dict=config_dict,
            provide_planning_service=False,
        )
        self.arm = self.moveit.get_planning_component(GROUP_NAME)
        self.psm = self.moveit.get_planning_scene_monitor()

        # Clientes contra move_group para el tramo cartesiano. Van en su propio
        # callback group porque grasp_candidate_cb bloquea esperando su resultado.
        self.client_cb_group = ReentrantCallbackGroup()
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path",
            callback_group=self.client_cb_group)
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory",
            callback_group=self.client_cb_group)

        self.status_pub = self.create_publisher(String, "/grasp_pick_status", 10)
        self.create_subscription(PoseStamped, "/grasp_candidate_pose",
                                 self.grasp_candidate_cb, 10,
                                 callback_group=self.cb_group)

        self.get_logger().info(
            f"grasp_pick_node iniciado (grupo='{GROUP_NAME}', tip='{TIP_LINK}', "
            f"approach_axis='{self.get_parameter('approach_axis').value}', "
            f"bias={self.get_parameter('bias').value}m)."
        )

    # ------------------------------------------------------------------ frames

    def _transform_to_base(self, pose_stamped: PoseStamped, target_frame="base_link") -> PoseStamped:
        if pose_stamped.header.frame_id == target_frame:
            return pose_stamped
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return do_transform_pose_stamped(pose_stamped, transform)
        except Exception as e:
            self.get_logger().error(
                f"Error al transformar frame {pose_stamped.header.frame_id} a {target_frame}: {e}")
            return None

    # ---------------------------------------------------------------------- IK

    def _compute_ik(self, pose_stamped: PoseStamped, seed_state: RobotState = None,
                    check_collision: bool = True, etiqueta: str = ""):
        timeout = self.get_parameter("ik_timeout").value
        restarts = self.get_parameter("ik_random_restarts").value

        ik_ok_pero_en_colision = False

        with self.psm.read_only() as scene:
            base_model = scene.current_state.robot_model
            jmg = base_model.get_joint_model_group(GROUP_NAME)

            state = RobotState(base_model)
            if seed_state is not None:
                state.joint_positions = seed_state.joint_positions
            else:
                state.joint_positions = scene.current_state.joint_positions
            state.update()

            if state.set_from_ik(GROUP_NAME, pose_stamped.pose, TIP_LINK, timeout):
                state.update()
                if not check_collision or scene.is_state_valid(state, GROUP_NAME):
                    return state, "ok"
                ik_ok_pero_en_colision = True

            for _ in range(restarts):
                state = RobotState(base_model)
                state.set_to_random_positions(jmg)
                state.update()
                if state.set_from_ik(GROUP_NAME, pose_stamped.pose, TIP_LINK, timeout):
                    state.update()
                    if not check_collision or scene.is_state_valid(state, GROUP_NAME):
                        return state, "ok"
                    ik_ok_pero_en_colision = True

        motivo = "colision" if ik_ok_pero_en_colision else "sin_ik"
        self.get_logger().warn(f"IK {etiqueta}: fallo por '{motivo}'.")
        return None, motivo

    # --------------------------------------------------------------- selección

    def select_reachable_grasp(self, candidates: list, d: float):
        eje = self.get_parameter("approach_axis").value
        angulos = [math.radians(a) for a in self.get_parameter("symmetry_angles_deg").value]
        skip_col_grasp = self.get_parameter("skip_collision_check_at_grasp").value

        for idx, cand in enumerate(candidates):
            for grados, g in _variantes_simetria(cand, eje, angulos):
                gpre = PoseStamped()
                gpre.header = g.header
                gpre.pose = _retroceder(g.pose, d, eje)

                q_gpre, motivo_pre = self._compute_ik(
                    gpre, etiqueta=f"cand{idx} rot{grados:+.0f}° (pre-agarre)")
                if q_gpre is None:
                    continue

                q_g, motivo_g = self._compute_ik(
                    g, seed_state=q_gpre,
                    check_collision=not skip_col_grasp,
                    etiqueta=f"cand{idx} rot{grados:+.0f}° (agarre)")
                if q_g is None:
                    continue

                self.get_logger().info(
                    f"Candidato {idx} con rotación {grados:+.0f}° sobre el eje "
                    f"'{eje}': alcanzable.")
                return g, gpre, q_g, q_gpre

        return None

    # --------------------------------------------------------------- ejecución

    def execute_segment_to_state(self, target_state: RobotState,
                                 cartesian: bool = False,
                                 target_pose: PoseStamped = None) -> str:
        """Ejecuta un tramo hasta `target_state`.

        Con `cartesian=True` el TCP sigue una recta hasta `target_pose` en vez de
        un camino arbitrario en espacio de joints; `target_pose` es obligatoria en
        ese caso y `target_state` se ignora.
        """
        if cartesian:
            status = self._plan_and_execute_cartesian(target_pose)
        else:
            status = self._plan_and_execute_ompl(target_state)

        if status == "exito":
            settle_time = self.get_parameter("settle_time_after_segment").value
            if settle_time > 0.0:
                time.sleep(settle_time)

        return status

    def _plan_and_execute_ompl(self, target_state: RobotState) -> str:
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=target_state)

        params = PlanRequestParameters(self.moveit, "")
        params.planning_pipeline = "ompl"
        params.planning_time = self.get_parameter("planning_time").value
        params.planning_attempts = self.get_parameter("planning_attempts").value

        plan_result = self.arm.plan(single_plan_parameters=params)

        if not plan_result or plan_result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = plan_result.error_code.val if plan_result else MoveItErrorCodes.FAILURE
            reason = get_error_string(code)
            self.get_logger().error(f"Fallo de planificación a RobotState (Código {code}): {reason}")
            return f"fallo_planificacion: {reason}"

        exec_status = self.moveit.execute(plan_result.trajectory, controllers=[])
        if not exec_status:
            self.get_logger().error("Ejecución rechazada por el controlador.")
            return "fallo_ejecucion"

        return "exito"

    def _plan_and_execute_cartesian(self, target_pose: PoseStamped) -> str:
        timeout = self.get_parameter("cartesian_timeout").value

        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servicio /compute_cartesian_path no disponible.")
            return "fallo_planificacion: /compute_cartesian_path no disponible"

        req = GetCartesianPath.Request()
        req.header.frame_id = target_pose.header.frame_id
        req.start_state.is_diff = True          # parte del estado actual del robot
        req.group_name = GROUP_NAME
        req.link_name = TIP_LINK
        req.waypoints = [target_pose.pose]
        req.max_step = self.get_parameter("cartesian_max_step").value
        req.avoid_collisions = self.get_parameter("cartesian_avoid_collisions").value

        done = threading.Event()
        future = self.cartesian_client.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            self.get_logger().error("Timeout esperando /compute_cartesian_path.")
            return "fallo_planificacion: timeout en /compute_cartesian_path"

        result = future.result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            reason = get_error_string(result.error_code.val)
            self.get_logger().error(f"Fallo de planificación cartesiana: {reason}")
            return f"fallo_planificacion: {reason}"

        min_fraction = self.get_parameter("cartesian_min_fraction").value
        if result.fraction < min_fraction:
            self.get_logger().error(
                f"Recta cartesiana incompleta: fraction={result.fraction:.3f} "
                f"(mínimo {min_fraction:.3f}). No se ejecuta.")
            return f"fallo_planificacion: cartesiano incompleto ({result.fraction:.3f})"

        self.get_logger().info(
            f"Recta cartesiana planificada (fraction={result.fraction:.3f}).")
        return self._execute_trajectory_msg(result.solution)

    def _execute_trajectory_msg(self, trajectory) -> str:
        timeout = self.get_parameter("cartesian_timeout").value

        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Acción /execute_trajectory no disponible.")
            return "fallo_ejecucion: /execute_trajectory no disponible"

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        sent = threading.Event()
        send_future = self.execute_client.send_goal_async(goal)
        send_future.add_done_callback(lambda _f: sent.set())
        if not sent.wait(timeout):
            self.get_logger().error("Timeout enviando el goal a /execute_trajectory.")
            return "fallo_ejecucion: timeout enviando el goal"

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rechazado por /execute_trajectory.")
            return "fallo_ejecucion: goal rechazado"

        finished = threading.Event()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda _f: finished.set())
        if not finished.wait(timeout):
            self.get_logger().error("Timeout esperando el resultado de /execute_trajectory.")
            return "fallo_ejecucion: timeout esperando el resultado"

        error_code = result_future.result().result.error_code
        if error_code.val != MoveItErrorCodes.SUCCESS:
            reason = get_error_string(error_code.val)
            self.get_logger().error(f"Fallo de ejecución cartesiana: {reason}")
            return f"fallo_ejecucion: {reason}"

        return "exito"

    # ---------------------------------------------------------------- callback

    def grasp_candidate_cb(self, msg: PoseStamped):
        d = self.get_parameter("approach_distance").value
        bias = self.get_parameter("bias").value
        eje = self.get_parameter("approach_axis").value

        self.get_logger().info(f"Pose de agarre recibida (frame={msg.header.frame_id}). Procesando...")

        qn = _q_normalize((msg.pose.orientation.x, msg.pose.orientation.y,
                           msg.pose.orientation.z, msg.pose.orientation.w))
        msg.pose.orientation.x, msg.pose.orientation.y = qn[0], qn[1]
        msg.pose.orientation.z, msg.pose.orientation.w = qn[2], qn[3]

        msg_base_link = self._transform_to_base(msg, target_frame="base_link")
        if msg_base_link is None:
            return

        # Aplicar el bias a lo largo del eje de aproximación si es distinto de cero
        if abs(bias) > 1e-6:
            msg_base_link.pose = _aplicar_bias(msg_base_link.pose, bias, eje)
            self.get_logger().info(f"Bias aplicado: {bias:+.3f}m en eje local '{eje}'.")

        p = msg_base_link.pose.position
        self.get_logger().info(
            f"Pose final en base_link (con bias): xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
            f"q=({qn[0]:.3f}, {qn[1]:.3f}, {qn[2]:.3f}, {qn[3]:.3f})")

        result = self.select_reachable_grasp([msg_base_link], d)

        if result is None:
            self.get_logger().warn("sin_agarre_alcanzable (revisa los motivos de IK arriba)")
            self.status_pub.publish(String(data="sin_agarre_alcanzable"))
            return

        g, gpre, q_g, q_gpre = result
        self.get_logger().info("Agarre válido encontrado. Tramo 1: aproximación (gpre)...")

        status = self.execute_segment_to_state(q_gpre)
        if not status.startswith("exito"):
            self.get_logger().error(f"Fallo en aproximación -> {status}")
            self.status_pub.publish(String(data=status))
            return

        self.get_logger().info("Aproximación completada. Tramo 2: agarre final (g), cartesiano...")

        status = self.execute_segment_to_state(q_g, cartesian=True, target_pose=g)
        self.get_logger().info(f"Resultado final del agarre: {status}")
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = GraspPickNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.moveit.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
