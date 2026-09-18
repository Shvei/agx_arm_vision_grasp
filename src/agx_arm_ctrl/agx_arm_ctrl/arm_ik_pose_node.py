#!/usr/bin/env python3
import math
import os
from typing import List, Optional

for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(key, "1")

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    import casadi
    import pinocchio as pin
    from pinocchio import casadi as cpin
except ImportError as exc:
    casadi = None
    pin = None
    cpin = None
    PINOCCHIO_IMPORT_ERROR = exc
else:
    PINOCCHIO_IMPORT_ERROR = None


def xyzrpy_to_mat(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    mat[:3, 3] = np.array([x, y, z])
    return mat


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def quat_xyzw_to_matrix(quat) -> np.ndarray:
    x, y, z, w = normalize_quaternion(np.array(quat, dtype=float))
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def matrix_to_quat_xyzw(mat: np.ndarray) -> np.ndarray:
    m = np.array(mat, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion(np.array([x, y, z, w], dtype=float))


def normalize_quaternion(quat: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def rotation_angle(mat: np.ndarray) -> float:
    value = (float(np.trace(mat)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, value)))


def make_pin_se3(mat: np.ndarray):
    se3 = pin.SE3.Identity()
    se3.rotation[:, :] = np.array(mat[:3, :3], dtype=float)
    se3.translation[:] = np.array(mat[:3, 3], dtype=float)
    return se3


def pose_to_mat(pose: Pose) -> np.ndarray:
    mat = np.eye(4)
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    mat[:3, :3] = quat_xyzw_to_matrix(quat)
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


def mat_to_pose(mat: np.ndarray) -> Pose:
    quat = matrix_to_quat_xyzw(mat[:3, :3])
    pose = Pose()
    pose.position.x = float(mat[0, 3])
    pose.position.y = float(mat[1, 3])
    pose.position.z = float(mat[2, 3])
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


class ArmIK:
    def __init__(
        self,
        urdf_path: str,
        package_dirs: List[str],
        locked_joints: List[str],
        ee_parent_joint: str,
        ee_frame_name: str,
        tool_pre_rot_rpy: List[float],
        tool_translation_xyz: List[float],
        collision_pairs_flat: List[int],
        w_pos: float,
        w_ori: float,
        w_reg: float,
        w_smooth: float,
        ipopt_max_iter: int,
        ipopt_tol: float,
    ):
        if pin is None or cpin is None or casadi is None:
            raise ImportError(
                "pinocchio/casadi import failed: %s" % PINOCCHIO_IMPORT_ERROR
            )

        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=package_dirs,
        )
        unique_locked_joints = self._deduplicate_locked_joints(locked_joints)
        if unique_locked_joints:
            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=unique_locked_joints,
                reference_configuration=np.zeros(self.robot.model.nq),
            )
        else:
            self.reduced_robot = self.robot

        tool_rot = xyzrpy_to_mat(
            0.0,
            0.0,
            0.0,
            tool_pre_rot_rpy[0],
            tool_pre_rot_rpy[1],
            tool_pre_rot_rpy[2],
        )
        tool_trans = xyzrpy_to_mat(
            tool_translation_xyz[0],
            tool_translation_xyz[1],
            tool_translation_xyz[2],
            0.0,
            0.0,
            0.0,
        )
        ee_mat = tool_rot @ tool_trans

        self.reduced_robot.model.addFrame(
            pin.Frame(
                ee_frame_name,
                self.reduced_robot.model.getJointId(ee_parent_joint),
                make_pin_se3(ee_mat),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.data = self.reduced_robot.model.createData()

        self.geom_model = self.reduced_robot.collision_model
        for index in range(0, len(collision_pairs_flat), 2):
            self.geom_model.addCollisionPair(
                pin.CollisionPair(
                    int(collision_pairs_flat[index]),
                    int(collision_pairs_flat[index + 1]),
                )
            )
        self.geometry_data = pin.GeometryData(self.geom_model)

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.ctf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        self.ee_id = self.reduced_robot.model.getFrameId(ee_frame_name)

        self.error = casadi.Function(
            "error",
            [self.cq, self.ctf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.ee_id].inverse() * cpin.SE3(self.ctf)
                    ).vector
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_q_prev = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]
        total_cost = casadi.sumsqr(w_pos * pos_error) + casadi.sumsqr(
            w_ori * ori_error
        )
        regularization = casadi.sumsqr(self.var_q)
        smooth_cost = casadi.sumsqr(self.var_q - self.param_q_prev)
        self.opti.minimize(total_cost + w_reg * regularization + w_smooth * smooth_cost)

        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        self.opti.solver(
            "ipopt",
            {
                "ipopt": {
                    "print_level": 0,
                    "max_iter": ipopt_max_iter,
                    "tol": ipopt_tol,
                },
                "print_time": False,
            },
        )

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)

    def _deduplicate_locked_joints(self, locked_joints: List[str]) -> List[str]:
        unique_names = []
        seen_joint_ids = set()
        for joint_name in locked_joints:
            try:
                joint_id = self.robot.model.getJointId(joint_name)
            except Exception:
                continue
            if joint_id <= 0 or joint_id in seen_joint_ids:
                continue
            seen_joint_ids.add(joint_id)
            unique_names.append(joint_name)
        return unique_names

    @property
    def nq(self) -> int:
        return self.reduced_robot.model.nq

    def active_joint_names(self) -> List[str]:
        return [name for name in self.reduced_robot.model.names if name != "universe"]

    def sync_state(self, q_current: List[float]) -> None:
        q = np.array(q_current, dtype=float)
        if q.shape[0] == self.nq:
            self.init_data = q
            self.history_data = q

    def solve(self, target_pose: np.ndarray) -> np.ndarray:
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_q_prev, self.history_data)
        self.opti.set_value(self.param_tf, target_pose)
        self.opti.solve_limited()
        sol_q = np.array(self.opti.value(self.var_q)).reshape(-1)
        self.init_data = sol_q
        self.history_data = sol_q
        return sol_q

    def get_pose_mat(self, q: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=float).reshape(-1)
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        return np.array(self.reduced_robot.data.oMf[self.ee_id].homogeneous)

    def check_self_collision(self, q: np.ndarray) -> bool:
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        pin.updateGeometryPlacements(
            self.reduced_robot.model,
            self.reduced_robot.data,
            self.geom_model,
            self.geometry_data,
        )
        return bool(pin.computeCollisions(self.geom_model, self.geometry_data, False))


class ArmIKPoseNode(Node):
    def __init__(self):
        super().__init__("arm_ik_pose_node")
        self._declare_parameters()
        self._load_parameters()

        package_path = get_package_share_directory(self.robot_description_package)
        urdf_path = os.path.join(package_path, self.urdf_relative_path)
        self.ik = ArmIK(
            urdf_path=urdf_path,
            package_dirs=[package_path],
            locked_joints=self.locked_joints,
            ee_parent_joint=self.ee_parent_joint,
            ee_frame_name=self.ee_frame_name,
            tool_pre_rot_rpy=self.tool_pre_rot_rpy,
            tool_translation_xyz=self.tool_translation_xyz,
            collision_pairs_flat=self.collision_pairs_flat,
            w_pos=self.w_pos,
            w_ori=self.w_ori,
            w_reg=self.w_reg,
            w_smooth=self.w_smooth,
            ipopt_max_iter=self.ipopt_max_iter,
            ipopt_tol=self.ipopt_tol,
        )

        if not self.output_joint_names:
            self.output_joint_names = self.ik.active_joint_names()
        if len(self.output_joint_names) != self.ik.nq:
            raise ValueError(
                "output_joint_names length %d does not match IK nq %d"
                % (len(self.output_joint_names), self.ik.nq)
            )

        self.current_q: Optional[np.ndarray] = None
        self.gripper_position: Optional[float] = None

        self.pub_joint = self.create_publisher(JointState, self.output_joint_topic, 10)
        self.pub_collision = self.create_publisher(
            Bool,
            f"{self.output_joint_topic}_collision",
            10,
        )
        self.pub_fk_pose = (
            self.create_publisher(PoseStamped, self.fk_pose_topic, 10)
            if self.fk_pose_topic
            else None
        )

        self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self.pose_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.feedback_joint_topic,
            self.feedback_joint_callback,
            10,
        )
        if self.gripper_feedback_topic:
            self.create_subscription(
                JointState,
                self.gripper_feedback_topic,
                self.gripper_feedback_callback,
                10,
            )

        self.get_logger().info(
            "IK pose node ready. input=%s output=%s fk=%s urdf=%s nq=%d"
            % (
                self.input_pose_topic,
                self.output_joint_topic,
                self.fk_pose_topic,
                urdf_path,
                self.ik.nq,
            )
        )

    def _declare_parameters(self):
        self.declare_parameter("robot_description_package", "agx_arm_description")
        self.declare_parameter(
            "urdf_relative_path",
            "agx_arm_urdf/piper/urdf/piper_description.urdf",
        )
        self.declare_parameter("locked_joints", [""])
        self.declare_parameter("ee_parent_joint", "joint6")
        self.declare_parameter("ee_frame_name", "ik_tcp")
        self.declare_parameter("tool_pre_rot_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("tool_translation_xyz", [0.0, 0.0, 0.13])
        self.declare_parameter("collision_pairs_flat_csv", "")
        self.declare_parameter("enable_collision_check", False)

        self.declare_parameter("w_pos", 20.0)
        self.declare_parameter("w_ori", 2.0)
        self.declare_parameter("w_reg", 0.01)
        self.declare_parameter("w_smooth", 0.2)
        self.declare_parameter("ipopt_max_iter", 80)
        self.declare_parameter("ipopt_tol", 1.0e-4)
        self.declare_parameter("max_position_error_m", 0.03)
        self.declare_parameter("max_orientation_error_rad", 0.6)
        self.declare_parameter("max_joint_step_rad", 0.8)
        self.declare_parameter("require_feedback", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("append_gripper_position", False)

        self.declare_parameter("input_pose_topic", "control/ik_pose")
        self.declare_parameter("feedback_joint_topic", "feedback/joint_states")
        self.declare_parameter("gripper_feedback_topic", "feedback/joint_states")
        self.declare_parameter("output_joint_topic", "control/move_j")
        self.declare_parameter("fk_pose_topic", "ik_fk_pose")
        self.declare_parameter(
            "output_joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )

    def _load_parameters(self):
        self.robot_description_package = self.get_parameter(
            "robot_description_package"
        ).value
        self.urdf_relative_path = self.get_parameter("urdf_relative_path").value
        self.locked_joints = [
            str(name)
            for name in self.get_parameter("locked_joints").value
            if str(name)
        ]
        self.ee_parent_joint = self.get_parameter("ee_parent_joint").value
        self.ee_frame_name = self.get_parameter("ee_frame_name").value
        self.tool_pre_rot_rpy = [
            float(v) for v in self.get_parameter("tool_pre_rot_rpy").value
        ]
        self.tool_translation_xyz = [
            float(v) for v in self.get_parameter("tool_translation_xyz").value
        ]
        collision_pairs_flat_csv = str(
            self.get_parameter("collision_pairs_flat_csv").value
        ).strip()
        self.collision_pairs_flat = self._parse_int_csv(collision_pairs_flat_csv)
        if len(self.collision_pairs_flat) % 2 != 0:
            raise ValueError("collision_pairs_flat length must be even")
        self.enable_collision_check = bool(
            self.get_parameter("enable_collision_check").value
        )

        self.w_pos = float(self.get_parameter("w_pos").value)
        self.w_ori = float(self.get_parameter("w_ori").value)
        self.w_reg = float(self.get_parameter("w_reg").value)
        self.w_smooth = float(self.get_parameter("w_smooth").value)
        self.ipopt_max_iter = int(self.get_parameter("ipopt_max_iter").value)
        self.ipopt_tol = float(self.get_parameter("ipopt_tol").value)
        self.max_position_error_m = float(
            self.get_parameter("max_position_error_m").value
        )
        self.max_orientation_error_rad = float(
            self.get_parameter("max_orientation_error_rad").value
        )
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.require_feedback = bool(self.get_parameter("require_feedback").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.append_gripper_position = bool(
            self.get_parameter("append_gripper_position").value
        )

        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.feedback_joint_topic = self.get_parameter("feedback_joint_topic").value
        self.gripper_feedback_topic = self.get_parameter("gripper_feedback_topic").value
        self.output_joint_topic = self.get_parameter("output_joint_topic").value
        self.fk_pose_topic = str(self.get_parameter("fk_pose_topic").value).strip()
        self.output_joint_names = [
            str(name)
            for name in self.get_parameter("output_joint_names").value
            if str(name)
        ]

    def _parse_int_csv(self, value: str) -> List[int]:
        if not value:
            return []
        return [int(item.strip()) for item in value.split(",") if item.strip()]

    def feedback_joint_callback(self, msg: JointState):
        joint_map = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        if not all(name in joint_map for name in self.output_joint_names):
            return
        q = np.array([joint_map[name] for name in self.output_joint_names], dtype=float)
        self.current_q = q
        self.ik.sync_state(q.tolist())
        if "gripper" in joint_map:
            self.gripper_position = float(joint_map["gripper"])

    def gripper_feedback_callback(self, msg: JointState):
        for index, name in enumerate(msg.name):
            if name == "gripper" and index < len(msg.position):
                self.gripper_position = float(msg.position[index])
                return

    def pose_callback(self, msg: PoseStamped):
        if self.require_feedback and self.current_q is None:
            self.get_logger().warn(
                "No feedback joint state yet; ignore IK target",
                throttle_duration_sec=2.0,
            )
            return

        target = pose_to_mat(msg.pose)
        try:
            sol_q = self.ik.solve(target)
        except Exception as exc:
            self.get_logger().warn("IK solve failed: %s" % exc)
            return

        position_error, orientation_error = self._compute_error(sol_q, target)
        if position_error > self.max_position_error_m:
            self.get_logger().warn(
                "Reject IK solution: position error %.4f > %.4f"
                % (position_error, self.max_position_error_m)
            )
            return
        if orientation_error > self.max_orientation_error_rad:
            self.get_logger().warn(
                "Reject IK solution: orientation error %.4f > %.4f"
                % (orientation_error, self.max_orientation_error_rad)
            )
            return

        if self.current_q is not None:
            max_step = float(np.max(np.abs(sol_q - self.current_q)))
            if max_step > self.max_joint_step_rad:
                self.get_logger().warn(
                    "Reject IK solution: max joint step %.4f > %.4f"
                    % (max_step, self.max_joint_step_rad)
                )
                return

        if self.enable_collision_check:
            collision = self.ik.check_self_collision(sol_q)
            col_msg = Bool()
            col_msg.data = collision
            self.pub_collision.publish(col_msg)
            if collision:
                self.get_logger().warn("Reject IK solution: self collision")
                return

        self._publish_solution(sol_q, msg.header.stamp, target)

    def _compute_error(self, q: np.ndarray, target: np.ndarray):
        fk = self.ik.get_pose_mat(q)
        position_error = float(np.linalg.norm(fk[:3, 3] - target[:3, 3]))
        orientation_error = rotation_angle(fk[:3, :3].T @ target[:3, :3])
        return position_error, orientation_error

    def _publish_solution(self, q: np.ndarray, stamp, target: np.ndarray):
        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = list(self.output_joint_names)
        joint_msg.position = q.tolist()

        if self.append_gripper_position and self.gripper_position is not None:
            joint_msg.name.append("gripper")
            joint_msg.position.append(self.gripper_position)

        fk = self.ik.get_pose_mat(q)
        if self.pub_fk_pose is not None:
            fk_msg = PoseStamped()
            fk_msg.header.stamp = stamp
            fk_msg.header.frame_id = "base_link"
            fk_msg.pose = mat_to_pose(fk)
            self.pub_fk_pose.publish(fk_msg)

        position_error = float(np.linalg.norm(fk[:3, 3] - target[:3, 3]))
        if self.dry_run:
            self.get_logger().info(
                "dry_run=true: IK solved but did not publish move_j, pos_error=%.4f"
                % position_error
            )
            return

        self.pub_joint.publish(joint_msg)
        self.get_logger().info(
            "Published move_j from IK, pos_error=%.4f, q=%s"
            % (
                position_error,
                ", ".join("%.3f" % value for value in q.tolist()),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmIKPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
