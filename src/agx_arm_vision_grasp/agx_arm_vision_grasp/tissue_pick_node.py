#!/usr/bin/env python3
import copy
import math
import threading
import time
from typing import Optional

from agx_arm_msgs.msg import AgxArmStatus
from geometry_msgs.msg import Point, PoseArray, PoseStamped
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from agx_arm_vision_grasp.geometry import (
    quaternion_from_euler,
    rotation_matrix_from_quaternion,
)


class TissuePickNode(Node):
    """Run a conservative open-loop grasp sequence from a detected target pose."""

    def __init__(self):
        super().__init__("tissue_pick")
        self._declare_parameters()
        self._load_parameters()

        self.latest_target: Optional[PoseStamped] = None
        self.latest_tcp: Optional[PoseStamped] = None
        self.latest_arm_status: Optional[AgxArmStatus] = None
        self.latest_target_rx_time = None
        self.lock = threading.Lock()
        self.busy = False
        self.auto_pick_started = False

        self.create_subscription(
            PoseStamped, self.target_pose_topic, self._target_callback, 10
        )
        self.create_subscription(PoseStamped, self.tcp_pose_topic, self._tcp_callback, 10)
        self.create_subscription(
            AgxArmStatus, self.arm_status_topic, self._arm_status_callback, 10
        )

        self.move_p_pub = self.create_publisher(PoseStamped, self.move_p_topic, 10)
        self.move_l_pub = self.create_publisher(PoseStamped, self.move_l_topic, 10)
        self.gripper_pub = self.create_publisher(JointState, self.gripper_topic, 10)
        self.pregrasp_pub = self.create_publisher(
            PoseStamped, self.pregrasp_pose_topic, 10
        )
        self.grasp_pub = self.create_publisher(PoseStamped, self.grasp_pose_topic, 10)
        self.lift_pub = self.create_publisher(PoseStamped, self.lift_pose_topic, 10)
        self.plan_pose_array_pub = self.create_publisher(
            PoseArray, self.plan_pose_array_topic, 10
        )
        self.plan_marker_pub = self.create_publisher(
            MarkerArray, self.plan_marker_topic, 10
        )

        self.create_service(Trigger, "pick_tissue", self._pick_service_callback)
        self.create_service(Trigger, "reset_home", self._reset_home_service_callback)
        self.create_timer(0.2, self._auto_pick_timer)
        if self.publish_plan_preview:
            self.create_timer(self.plan_preview_period_sec, self._plan_preview_timer)

        self.get_logger().info(
            "Tissue pick node ready. dry_run=%s, trigger pick with: ros2 service call /pick_tissue std_srvs/srv/Trigger {}, reset with: ros2 service call /reset_home std_srvs/srv/Trigger {}"
            % self.dry_run
        )

    def _declare_parameters(self):
        self.declare_parameter("target_pose_topic", "vision/tissue_pose")
        self.declare_parameter("tcp_pose_topic", "feedback/tcp_pose")
        self.declare_parameter("arm_status_topic", "feedback/arm_status")
        self.declare_parameter("move_p_topic", "control/move_p")
        self.declare_parameter("move_l_topic", "control/move_l")
        self.declare_parameter("gripper_topic", "control/joint_states")
        self.declare_parameter("pregrasp_pose_topic", "vision/pregrasp_pose")
        self.declare_parameter("grasp_pose_topic", "vision/grasp_pose")
        self.declare_parameter("lift_pose_topic", "vision/lift_pose")
        self.declare_parameter("plan_pose_array_topic", "vision/grasp_plan")
        self.declare_parameter("plan_marker_topic", "vision/grasp_plan_markers")
        self.declare_parameter("publish_plan_preview", True)
        self.declare_parameter("plan_preview_period_sec", 0.2)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("auto_pick", False)
        self.declare_parameter("return_home_after_pick", False)
        self.declare_parameter(
            "home_tcp_pose",
            [
                0.17901399568224927,
                0.012838163267257775,
                0.1465926854184608,
                -0.05364161383419479,
                0.7804786122100572,
                0.003642533921323063,
                0.6228663139828307,
            ],
        )
        self.declare_parameter("keep_current_orientation", True)
        self.declare_parameter("orientation_mode", "current")
        self.declare_parameter("fixed_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("approach_mode", "tcp_axis")
        self.declare_parameter("approach_axis_tcp", [0.0, 0.0, 1.0])
        self.declare_parameter("approach_axis_base", [0.0, 0.0, -1.0])
        self.declare_parameter("approach_distance_m", 0.10)
        self.declare_parameter("grasp_depth_offset_m", 0.015)
        self.declare_parameter("min_pregrasp_z_m", 0.0)
        self.declare_parameter("min_grasp_z_m", 0.0)
        self.declare_parameter("lift_offset_base", [0.0, 0.0, 0.08])
        self.declare_parameter("object_to_grasp_offset_base", [0.0, 0.0, 0.0])
        self.declare_parameter("open_width_m", 0.085)
        self.declare_parameter("close_width_m", 0.018)
        self.declare_parameter("gripper_effort_n", 1.5)
        self.declare_parameter("motion_timeout_sec", 8.0)
        self.declare_parameter("settle_time_sec", 1.5)
        self.declare_parameter("pose_tolerance_m", 0.025)
        self.declare_parameter("require_recent_target_sec", 1.0)
        self.declare_parameter("abort_on_motion_timeout", True)
        self.declare_parameter("abort_on_arm_error", True)
        self.declare_parameter("workspace_min", [0.05, -0.45, 0.02])
        self.declare_parameter("workspace_max", [0.65, 0.45, 0.60])
        self.declare_parameter("use_feedback_wait", True)
        self.declare_parameter("use_linear_approach", True)

    def _load_parameters(self):
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.tcp_pose_topic = self.get_parameter("tcp_pose_topic").value
        self.arm_status_topic = self.get_parameter("arm_status_topic").value
        self.move_p_topic = self.get_parameter("move_p_topic").value
        self.move_l_topic = self.get_parameter("move_l_topic").value
        self.gripper_topic = self.get_parameter("gripper_topic").value
        self.pregrasp_pose_topic = self.get_parameter("pregrasp_pose_topic").value
        self.grasp_pose_topic = self.get_parameter("grasp_pose_topic").value
        self.lift_pose_topic = self.get_parameter("lift_pose_topic").value
        self.plan_pose_array_topic = self.get_parameter("plan_pose_array_topic").value
        self.plan_marker_topic = self.get_parameter("plan_marker_topic").value
        self.publish_plan_preview = bool(
            self.get_parameter("publish_plan_preview").value
        )
        self.plan_preview_period_sec = max(
            0.05, float(self.get_parameter("plan_preview_period_sec").value)
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.auto_pick = bool(self.get_parameter("auto_pick").value)
        self.return_home_after_pick = bool(
            self.get_parameter("return_home_after_pick").value
        )
        self.home_tcp_pose = self._as_vector(
            "home_tcp_pose",
            7,
            [
                0.17901399568224927,
                0.012838163267257775,
                0.1465926854184608,
                -0.05364161383419479,
                0.7804786122100572,
                0.003642533921323063,
                0.6228663139828307,
            ],
        )
        self.keep_current_orientation = bool(
            self.get_parameter("keep_current_orientation").value
        )
        self.orientation_mode = str(self.get_parameter("orientation_mode").value)
        self.fixed_orientation_xyzw = self._as_vector(
            "fixed_orientation_xyzw", 4, [0.0, 0.0, 0.0, 1.0]
        )
        self.approach_mode = str(self.get_parameter("approach_mode").value)
        self.approach_axis_tcp = self._normalize(
            self._as_vector("approach_axis_tcp", 3, [0.0, 0.0, 1.0])
        )
        self.approach_axis_base = self._normalize(
            self._as_vector("approach_axis_base", 3, [0.0, 0.0, -1.0])
        )
        self.approach_distance_m = float(
            self.get_parameter("approach_distance_m").value
        )
        self.grasp_depth_offset_m = float(
            self.get_parameter("grasp_depth_offset_m").value
        )
        self.min_pregrasp_z_m = float(self.get_parameter("min_pregrasp_z_m").value)
        self.min_grasp_z_m = float(self.get_parameter("min_grasp_z_m").value)
        self.lift_offset_base = self._as_vector("lift_offset_base", 3, [0.0, 0.0, 0.08])
        self.object_to_grasp_offset_base = self._as_vector(
            "object_to_grasp_offset_base", 3, [0.0, 0.0, 0.0]
        )
        self.open_width_m = float(self.get_parameter("open_width_m").value)
        self.close_width_m = float(self.get_parameter("close_width_m").value)
        self.gripper_effort_n = float(self.get_parameter("gripper_effort_n").value)
        self.motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self.settle_time_sec = float(self.get_parameter("settle_time_sec").value)
        self.pose_tolerance_m = float(self.get_parameter("pose_tolerance_m").value)
        self.require_recent_target_sec = float(
            self.get_parameter("require_recent_target_sec").value
        )
        self.abort_on_motion_timeout = bool(
            self.get_parameter("abort_on_motion_timeout").value
        )
        self.abort_on_arm_error = bool(self.get_parameter("abort_on_arm_error").value)
        self.workspace_min = self._as_vector("workspace_min", 3, [0.05, -0.45, 0.02])
        self.workspace_max = self._as_vector("workspace_max", 3, [0.65, 0.45, 0.60])
        self.use_feedback_wait = bool(self.get_parameter("use_feedback_wait").value)
        self.use_linear_approach = bool(self.get_parameter("use_linear_approach").value)

    def _as_vector(self, name: str, size: int, default):
        value = self.get_parameter(name).value
        try:
            result = np.array([float(v) for v in value], dtype=np.float64)
        except TypeError:
            result = np.array(default, dtype=np.float64)
        if result.size != size:
            self.get_logger().warn(
                f"Parameter {name} must have {size} values, using {default}"
            )
            result = np.array(default, dtype=np.float64)
        return result

    def _normalize(self, value: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(value))
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=np.float64)
        return value / norm

    def _target_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_target = msg
            self.latest_target_rx_time = self.get_clock().now()

    def _tcp_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_tcp = msg

    def _arm_status_callback(self, msg: AgxArmStatus):
        with self.lock:
            self.latest_arm_status = msg

    def _pick_service_callback(self, request, response):
        del request
        started, message = self._start_pick_thread("service")
        if not started:
            self.get_logger().warn("Reject pick request: %s" % message)
        response.success = started
        response.message = message
        return response

    def _reset_home_service_callback(self, request, response):
        del request
        started, message = self._start_home_thread("service")
        if not started:
            self.get_logger().warn("Reject reset request: %s" % message)
        response.success = started
        response.message = message
        return response

    def _auto_pick_timer(self):
        if not self.auto_pick or self.auto_pick_started:
            return
        started, message = self._start_pick_thread("auto")
        if started:
            self.auto_pick_started = True
            self.get_logger().info(message)

    def _plan_preview_timer(self):
        plan = self._build_grasp_plan(quiet=True)
        if plan is None:
            return
        self._publish_grasp_plan(plan)

    def _start_pick_thread(self, source: str):
        with self.lock:
            if self.busy:
                return False, "pick sequence is already running"
            if not self._has_recent_target_locked():
                return False, "no recent tissue target pose"
            if self.abort_on_arm_error and self.latest_arm_status is None:
                return False, "no arm status feedback yet"
            if self._has_arm_error_locked():
                status = int(self.latest_arm_status.arm_status)
                return (
                    False,
                    "arm_status=%d; call /clear_error before picking" % status,
                )
            self.busy = True

        worker = threading.Thread(target=self._run_pick_sequence, args=(source,))
        worker.daemon = True
        worker.start()
        return True, f"started tissue pick sequence from {source}"

    def _start_home_thread(self, source: str):
        with self.lock:
            if self.busy:
                return False, "pick/reset sequence is already running"
            self.busy = True

        worker = threading.Thread(target=self._run_home_sequence, args=(source,))
        worker.daemon = True
        worker.start()
        return True, f"started home reset sequence from {source}"

    def _has_recent_target_locked(self) -> bool:
        if self.latest_target is None or self.latest_target_rx_time is None:
            return False
        age = (self.get_clock().now() - self.latest_target_rx_time).nanoseconds / 1e9
        return age <= self.require_recent_target_sec

    def _has_arm_error_locked(self) -> bool:
        return (
            self.abort_on_arm_error
            and self.latest_arm_status is not None
            and int(self.latest_arm_status.arm_status) != 0
        )

    def _run_pick_sequence(self, source: str):
        try:
            plan = self._build_grasp_plan()
            if plan is None:
                return

            pregrasp, grasp, lift = plan
            self._publish_grasp_plan(plan)

            if self.dry_run:
                self.get_logger().warn(
                    "dry_run=true: planned pick from %s but did not publish arm/gripper commands"
                    % source
                )
                return

            self._command_gripper(self.open_width_m)
            time.sleep(0.4)

            self._command_pose(pregrasp, self.move_p_pub, "pregrasp")
            if not self._wait_motion(pregrasp) and self.abort_on_motion_timeout:
                self.get_logger().error("Abort pick: pregrasp pose was not reached")
                return

            approach_pub = self.move_l_pub if self.use_linear_approach else self.move_p_pub
            self._command_pose(grasp, approach_pub, "grasp")
            if not self._wait_motion(grasp) and self.abort_on_motion_timeout:
                self.get_logger().error("Abort pick: grasp pose was not reached")
                return

            self._command_gripper(self.close_width_m)
            time.sleep(0.8)

            self._command_pose(lift, approach_pub, "lift")
            if not self._wait_motion(lift) and self.abort_on_motion_timeout:
                self.get_logger().error("Abort pick: lift pose was not reached")
                return

            if self.return_home_after_pick:
                home = self._make_home_pose()
                if home is not None:
                    self._command_pose(home, self.move_p_pub, "home")
                    if not self._wait_motion(home) and self.abort_on_motion_timeout:
                        self.get_logger().error("Abort pick: home pose was not reached")
                        return

            self.get_logger().info("Tissue pick sequence finished")
        finally:
            with self.lock:
                self.busy = False

    def _run_home_sequence(self, source: str):
        try:
            home = self._make_home_pose()
            if home is None:
                return

            if self.dry_run:
                self.get_logger().warn(
                    "dry_run=true: planned home reset from %s but did not publish arm command"
                    % source
                )
                return

            self._command_pose(home, self.move_p_pub, "home")
            if not self._wait_motion(home) and self.abort_on_motion_timeout:
                self.get_logger().error("Abort reset: home pose was not reached")
                return
            self.get_logger().info("Home reset sequence finished")
        finally:
            with self.lock:
                self.busy = False

    def _build_grasp_plan(self, quiet: bool = False):
        with self.lock:
            if not self._has_recent_target_locked():
                if not quiet:
                    self.get_logger().warn("No recent target pose; abort pick")
                return None
            target = copy.deepcopy(self.latest_target)
            tcp = copy.deepcopy(self.latest_tcp)

        orientation = self._select_orientation(target, tcp)
        approach_axis_base = self._approach_axis_base(orientation)

        target_p = self._position_to_np(target)
        grasp_p = (
            target_p
            + self.object_to_grasp_offset_base
            + approach_axis_base * self.grasp_depth_offset_m
        )
        if self.min_grasp_z_m > 0.0 and grasp_p[2] < self.min_grasp_z_m:
            if not quiet:
                self.get_logger().warn(
                    "Raised grasp z from %.3f to min_grasp_z_m %.3f"
                    % (grasp_p[2], self.min_grasp_z_m)
                )
            grasp_p[2] = self.min_grasp_z_m
        pregrasp_p = grasp_p - approach_axis_base * self.approach_distance_m
        if self.min_pregrasp_z_m > 0.0 and pregrasp_p[2] < self.min_pregrasp_z_m:
            if not quiet:
                self.get_logger().warn(
                    "Raised pregrasp z from %.3f to min_pregrasp_z_m %.3f"
                    % (pregrasp_p[2], self.min_pregrasp_z_m)
                )
            pregrasp_p[2] = self.min_pregrasp_z_m
        lift_p = grasp_p + self.lift_offset_base

        for name, point in (
            ("pregrasp", pregrasp_p),
            ("grasp", grasp_p),
            ("lift", lift_p),
        ):
            if not self._inside_workspace(point):
                if not quiet:
                    self.get_logger().warn(
                        "%s pose [%.3f, %.3f, %.3f] is outside workspace; abort pick"
                        % (name, point[0], point[1], point[2])
                    )
                return None

        pregrasp = self._make_pose(target.header.frame_id, pregrasp_p, orientation)
        grasp = self._make_pose(target.header.frame_id, grasp_p, orientation)
        lift = self._make_pose(target.header.frame_id, lift_p, orientation)
        return pregrasp, grasp, lift

    def _publish_grasp_plan(self, plan):
        pregrasp, grasp, lift = plan
        stamp = self.get_clock().now().to_msg()
        for pose in (pregrasp, grasp, lift):
            pose.header.stamp = stamp

        self.pregrasp_pub.publish(pregrasp)
        self.grasp_pub.publish(grasp)
        self.lift_pub.publish(lift)

        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = pregrasp.header.frame_id
        poses.poses = [pregrasp.pose, grasp.pose, lift.pose]
        self.plan_pose_array_pub.publish(poses)
        self.plan_marker_pub.publish(
            self._make_plan_markers(pregrasp, grasp, lift, stamp)
        )

    def _make_plan_markers(self, pregrasp, grasp, lift, stamp) -> MarkerArray:
        frame_id = pregrasp.header.frame_id
        markers = MarkerArray()
        labels = (
            ("pregrasp", pregrasp, (0.0, 0.8, 0.2, 0.9)),
            ("grasp", grasp, (0.95, 0.15, 0.05, 0.9)),
            ("lift", lift, (0.1, 0.35, 1.0, 0.9)),
        )

        for index, (name, pose, color) in enumerate(labels):
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = stamp
            sphere.ns = "tissue_grasp_plan_points"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose.pose
            sphere.scale.x = 0.025
            sphere.scale.y = 0.025
            sphere.scale.z = 0.025
            sphere.color.r = color[0]
            sphere.color.g = color[1]
            sphere.color.b = color[2]
            sphere.color.a = color[3]
            sphere.lifetime.sec = 1
            markers.markers.append(sphere)

            text = Marker()
            text.header.frame_id = frame_id
            text.header.stamp = stamp
            text.ns = "tissue_grasp_plan_labels"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = copy.deepcopy(pose.pose)
            text.pose.position.z += 0.035
            text.scale.z = 0.035
            text.color.r = color[0]
            text.color.g = color[1]
            text.color.b = color[2]
            text.color.a = 1.0
            text.text = name
            text.lifetime.sec = 1
            markers.markers.append(text)

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = "tissue_grasp_plan_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.008
        line.color.r = 1.0
        line.color.g = 0.8
        line.color.b = 0.05
        line.color.a = 0.9
        line.points = [
            self._pose_to_point(pregrasp),
            self._pose_to_point(grasp),
            self._pose_to_point(lift),
        ]
        line.lifetime.sec = 1
        markers.markers.append(line)
        return markers

    def _pose_to_point(self, pose: PoseStamped) -> Point:
        point = Point()
        point.x = pose.pose.position.x
        point.y = pose.pose.position.y
        point.z = pose.pose.position.z
        return point

    def _make_home_pose(self) -> Optional[PoseStamped]:
        home_p = self.home_tcp_pose[:3]
        if not self._inside_workspace(home_p):
            self.get_logger().warn(
                "home pose [%.3f, %.3f, %.3f] is outside workspace; abort reset"
                % (home_p[0], home_p[1], home_p[2])
            )
            return None
        return self._make_pose("base_link", home_p, self.home_tcp_pose[3:])

    def _select_orientation(self, target: PoseStamped, tcp: Optional[PoseStamped]):
        if self.orientation_mode == "top_down_current_yaw":
            if tcp is None:
                self.get_logger().warn(
                    "No tcp feedback yet; using fixed_orientation_xyzw",
                    throttle_duration_sec=2.0,
                )
                return self.fixed_orientation_xyzw.tolist()
            current_q = [
                tcp.pose.orientation.x,
                tcp.pose.orientation.y,
                tcp.pose.orientation.z,
                tcp.pose.orientation.w,
            ]
            current_yaw = self._yaw_from_quaternion(current_q)
            return quaternion_from_euler(-math.pi, 0.0, current_yaw).tolist()
        if self.orientation_mode == "fixed":
            return self.fixed_orientation_xyzw.tolist()
        if self.orientation_mode not in ("current", ""):
            self.get_logger().warn(
                "Unknown orientation_mode '%s', falling back to keep_current_orientation"
                % self.orientation_mode,
                throttle_duration_sec=2.0,
            )
        if self.keep_current_orientation and tcp is not None:
            return [
                tcp.pose.orientation.x,
                tcp.pose.orientation.y,
                tcp.pose.orientation.z,
                tcp.pose.orientation.w,
            ]
        if self.keep_current_orientation and tcp is None:
            self.get_logger().warn(
                "No tcp feedback yet; using fixed_orientation_xyzw",
                throttle_duration_sec=2.0,
            )
        if not self.keep_current_orientation:
            return self.fixed_orientation_xyzw.tolist()
        return [
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ]

    def _yaw_from_quaternion(self, orientation_xyzw) -> float:
        rot = rotation_matrix_from_quaternion(orientation_xyzw)
        return math.atan2(float(rot[1, 0]), float(rot[0, 0]))

    def _approach_axis_base(self, orientation_xyzw) -> np.ndarray:
        if self.approach_mode == "base_axis":
            return self.approach_axis_base
        if self.approach_mode != "tcp_axis":
            self.get_logger().warn(
                "Unknown approach_mode '%s', using tcp_axis" % self.approach_mode,
                throttle_duration_sec=2.0,
            )
        rot = rotation_matrix_from_quaternion(orientation_xyzw)
        return self._normalize(rot.dot(self.approach_axis_tcp))

    def _position_to_np(self, pose: PoseStamped) -> np.ndarray:
        return np.array(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            dtype=np.float64,
        )

    def _inside_workspace(self, point: np.ndarray) -> bool:
        return bool(np.all(point >= self.workspace_min) and np.all(point <= self.workspace_max))

    def _make_pose(self, frame_id: str, position: np.ndarray, orientation_xyzw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.x = float(orientation_xyzw[0])
        pose.pose.orientation.y = float(orientation_xyzw[1])
        pose.pose.orientation.z = float(orientation_xyzw[2])
        pose.pose.orientation.w = float(orientation_xyzw[3])
        return pose

    def _command_gripper(self, width_m: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(width_m)]
        msg.velocity = []
        msg.effort = [float(self.gripper_effort_n)]
        self.gripper_pub.publish(msg)

    def _command_pose(self, pose: PoseStamped, publisher, label: str):
        command = copy.deepcopy(pose)
        command.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            "%s -> [%.3f, %.3f, %.3f]"
            % (
                label,
                command.pose.position.x,
                command.pose.position.y,
                command.pose.position.z,
            )
        )
        for _ in range(3):
            publisher.publish(command)
            time.sleep(0.05)

    def _wait_motion(self, target: PoseStamped):
        if not self.use_feedback_wait:
            time.sleep(self.settle_time_sec)
            return True

        start = time.time()
        while time.time() - start < self.motion_timeout_sec:
            with self.lock:
                tcp = copy.deepcopy(self.latest_tcp)
                arm_status = copy.deepcopy(self.latest_arm_status)
            if (
                self.abort_on_arm_error
                and arm_status is not None
                and int(arm_status.arm_status) != 0
            ):
                self.get_logger().error(
                    "Arm reported arm_status=%d while waiting for motion"
                    % int(arm_status.arm_status)
                )
                return False
            if tcp is not None and (
                not tcp.header.frame_id or tcp.header.frame_id == target.header.frame_id
            ):
                dist = self._distance(tcp, target)
                if dist <= self.pose_tolerance_m:
                    return True
            time.sleep(0.05)

        self.get_logger().warn(
            "Timed out waiting for TCP feedback to reach target"
        )
        time.sleep(self.settle_time_sec)
        return False

    def _distance(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(args=None):
    rclpy.init(args=args)
    node = TissuePickNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
