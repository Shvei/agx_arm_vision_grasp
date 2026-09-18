#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import cv2
from geometry_msgs.msg import PoseStamped, TransformStamped
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from tf2_ros import TransformException
from agx_arm_vision_grasp.geometry import (
    quaternion_from_euler,
    quaternion_multiply,
    rotation_matrix_from_quaternion,
)


@dataclass
class TissueCandidate:
    contour: np.ndarray
    rect: tuple
    area: float
    depth_m: float
    valid_depth_ratio: float
    rectangularity: float
    aspect_ratio: float


class TissueDetector(Node):
    """Detect the blue tissue packet from aligned RGB-D images."""

    def __init__(self):
        super().__init__("tissue_detector")
        self._declare_parameters()
        self._load_parameters()

        self.camera_info: Optional[CameraInfo] = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.pose_camera_pub = self.create_publisher(
            PoseStamped, self.pose_camera_topic, 10
        )
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
        self.detected_image_pub = self.create_publisher(
            Image, self.detected_image_topic, 1
        )

        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, 10
        )

        color_sub = message_filters.Subscriber(self, Image, self.color_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=8,
            slop=0.08,
        )
        self.sync.registerCallback(self._image_callback)

        self.get_logger().info(
            "Tissue detector listening to color=%s depth=%s info=%s"
            % (self.color_topic, self.depth_topic, self.camera_info_topic)
        )

    def _declare_parameters(self):
        self.declare_parameter("color_topic", "camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "camera/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tissue_tf_parent_frame", "base_link")
        self.declare_parameter("tissue_frame_id", "tissue_target")
        self.declare_parameter("pose_topic", "vision/tissue_pose")
        self.declare_parameter("pose_camera_topic", "vision/tissue_pose_camera")
        self.declare_parameter("debug_image_topic", "vision/tissue_debug/image")
        self.declare_parameter("detected_image_topic", "vision/tissue_detected/image")
        self.declare_parameter("publish_tissue_tf", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_detected_image", True)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.12)
        self.declare_parameter("max_depth_m", 0.85)
        self.declare_parameter("hsv_h_min", 90)
        self.declare_parameter("hsv_h_max", 115)
        self.declare_parameter("hsv_s_min", 60)
        self.declare_parameter("hsv_s_max", 255)
        self.declare_parameter("hsv_v_min", 60)
        self.declare_parameter("hsv_v_max", 255)
        self.declare_parameter("min_area_px", 2500.0)
        self.declare_parameter("max_area_fraction", 0.65)
        self.declare_parameter("min_aspect_ratio", 0.55)
        self.declare_parameter("max_aspect_ratio", 4.0)
        self.declare_parameter("min_rectangularity", 0.55)
        self.declare_parameter("min_valid_depth_ratio", 0.35)
        self.declare_parameter("min_blue_fraction", 0.45)
        self.declare_parameter("target_center_offset_px", [0.0, 0.0])
        self.declare_parameter("morphology_kernel_px", 7)
        self.declare_parameter("tf_timeout_sec", 0.08)

    def _load_parameters(self):
        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.tissue_tf_parent_frame = self.get_parameter("tissue_tf_parent_frame").value
        self.tissue_frame_id = self.get_parameter("tissue_frame_id").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.pose_camera_topic = self.get_parameter("pose_camera_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.detected_image_topic = self.get_parameter("detected_image_topic").value
        self.publish_tissue_tf = bool(self.get_parameter("publish_tissue_tf").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )
        self.publish_detected_image = bool(
            self.get_parameter("publish_detected_image").value
        )
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.hsv_low = np.array(
            [
                int(self.get_parameter("hsv_h_min").value),
                int(self.get_parameter("hsv_s_min").value),
                int(self.get_parameter("hsv_v_min").value),
            ],
            dtype=np.uint8,
        )
        self.hsv_high = np.array(
            [
                int(self.get_parameter("hsv_h_max").value),
                int(self.get_parameter("hsv_s_max").value),
                int(self.get_parameter("hsv_v_max").value),
            ],
            dtype=np.uint8,
        )
        self.min_area_px = float(self.get_parameter("min_area_px").value)
        self.max_area_fraction = float(self.get_parameter("max_area_fraction").value)
        self.min_aspect_ratio = float(self.get_parameter("min_aspect_ratio").value)
        self.max_aspect_ratio = float(self.get_parameter("max_aspect_ratio").value)
        self.min_rectangularity = float(
            self.get_parameter("min_rectangularity").value
        )
        self.min_valid_depth_ratio = float(
            self.get_parameter("min_valid_depth_ratio").value
        )
        self.min_blue_fraction = float(self.get_parameter("min_blue_fraction").value)
        self.target_center_offset_px = self._load_vector_parameter(
            "target_center_offset_px", 2, [0.0, 0.0]
        )
        self.morphology_kernel_px = int(
            self.get_parameter("morphology_kernel_px").value
        )
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)

    def _load_vector_parameter(self, name: str, size: int, default: list) -> np.ndarray:
        value = self.get_parameter(name).value
        try:
            vector = np.array([float(v) for v in value], dtype=np.float64)
        except TypeError:
            vector = np.array(default, dtype=np.float64)
        if vector.size != size:
            self.get_logger().warn(
                f"Parameter {name} must have {size} values, using {default}"
            )
            return np.array(default, dtype=np.float64)
        return vector

    def _camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _image_callback(self, color_msg: Image, depth_msg: Image):
        if self.camera_info is None:
            self.get_logger().warn("Waiting for camera_info", throttle_duration_sec=2.0)
            return

        try:
            color = self._color_image_to_bgr(color_msg)
            depth_raw = self._depth_image_to_array(depth_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert images: {exc}")
            return

        depth_m = self._depth_to_meters(depth_raw)
        mask = self._build_mask(color, depth_m)
        candidate = self._select_candidate(mask, depth_m)

        debug = color.copy() if self.publish_debug_image else None
        if candidate is None:
            if debug is not None:
                self._publish_debug(debug, color_msg.header)
            if self.publish_detected_image:
                self._publish_image(self.detected_image_pub, color, color_msg.header)
            return

        pose_camera = self._candidate_to_pose(candidate, color_msg.header)
        self.pose_camera_pub.publish(pose_camera)

        pose_target = None
        if self.target_frame:
            pose_target = self._transform_pose(pose_camera, self.target_frame)
            if pose_target is not None:
                self.pose_pub.publish(pose_target)
        else:
            pose_target = pose_camera
            self.pose_pub.publish(pose_camera)

        if self.publish_tissue_tf:
            self._publish_tissue_tf(pose_camera, pose_target)

        if debug is not None:
            self._draw_debug(debug, candidate)
            self._publish_image(self.debug_pub, debug, color_msg.header)

        if self.publish_detected_image:
            detected = color.copy()
            self._draw_debug(detected, candidate)
            self._publish_image(self.detected_image_pub, detected, color_msg.header)

    def _color_image_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in ("rgb8", "bgr8"):
            row = data.reshape((msg.height, msg.step))
            image = row[:, : msg.width * 3].reshape((msg.height, msg.width, 3))
            if encoding == "rgb8":
                image = image[:, :, ::-1]
            return np.ascontiguousarray(image)
        if encoding in ("rgba8", "bgra8"):
            row = data.reshape((msg.height, msg.step))
            image = row[:, : msg.width * 4].reshape((msg.height, msg.width, 4))
            image = image[:, :, :3]
            if encoding == "rgba8":
                image = image[:, :, ::-1]
            return np.ascontiguousarray(image)
        raise ValueError(f"Unsupported color image encoding: {msg.encoding}")

    def _depth_image_to_array(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            dtype = np.dtype(np.uint16)
        elif encoding == "32fc1":
            dtype = np.dtype(np.float32)
        else:
            raise ValueError(f"Unsupported depth image encoding: {msg.encoding}")

        if msg.is_bigendian:
            dtype = dtype.newbyteorder(">")
        data = np.frombuffer(msg.data, dtype=dtype)
        columns = msg.step // dtype.itemsize
        return np.ascontiguousarray(data.reshape((msg.height, columns))[:, : msg.width])

    def _depth_to_meters(self, depth_raw: np.ndarray) -> np.ndarray:
        if depth_raw.ndim == 3:
            depth_raw = depth_raw[:, :, 0]
        if depth_raw.dtype == np.uint16:
            return depth_raw.astype(np.float32) * self.depth_scale
        return depth_raw.astype(np.float32)

    def _build_mask(self, color: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        depth_mask = (
            np.isfinite(depth_m)
            & (depth_m >= self.min_depth_m)
            & (depth_m <= self.max_depth_m)
        ).astype(np.uint8) * 255
        mask = cv2.bitwise_and(color_mask, depth_mask)

        kernel_size = max(1, self.morphology_kernel_px)
        if kernel_size > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (kernel_size, kernel_size)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _select_candidate(
        self, mask: np.ndarray, depth_m: np.ndarray
    ) -> Optional[TissueCandidate]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        image_area = float(mask.shape[0] * mask.shape[1])
        best = None
        best_score = -1.0

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area_px or area > image_area * self.max_area_fraction:
                continue

            rect = cv2.minAreaRect(contour)
            width, height = rect[1]
            if width <= 1.0 or height <= 1.0:
                continue
            aspect_ratio = max(width, height) / min(width, height)
            if not (self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio):
                continue

            rect_area = float(width * height)
            rectangularity = area / rect_area if rect_area > 0.0 else 0.0
            if rectangularity < self.min_rectangularity:
                continue
            if rectangularity < self.min_blue_fraction:
                continue

            contour_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
            valid_depth_mask = (
                (contour_mask > 0)
                & np.isfinite(depth_m)
                & (depth_m >= self.min_depth_m)
                & (depth_m <= self.max_depth_m)
            )
            contour_pixels = max(1, int(np.count_nonzero(contour_mask)))
            valid_depth_ratio = float(np.count_nonzero(valid_depth_mask)) / float(
                contour_pixels
            )
            if valid_depth_ratio < self.min_valid_depth_ratio:
                continue

            depths = depth_m[valid_depth_mask]
            depth = float(np.median(depths))
            score = area * rectangularity * valid_depth_ratio
            if score > best_score:
                best_score = score
                best = TissueCandidate(
                    contour=contour,
                    rect=rect,
                    area=area,
                    depth_m=depth,
                    valid_depth_ratio=valid_depth_ratio,
                    rectangularity=rectangularity,
                    aspect_ratio=aspect_ratio,
                )

        return best

    def _candidate_to_pose(self, candidate: TissueCandidate, header) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = header.frame_id or self.camera_frame

        center_px = candidate.rect[0]
        target_u = center_px[0] + self.target_center_offset_px[0]
        target_v = center_px[1] + self.target_center_offset_px[1]
        x, y, z = self._deproject(target_u, target_v, candidate.depth_m)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        width, height = candidate.rect[1]
        angle_deg = candidate.rect[2]
        if width < height:
            angle_deg += 90.0
        q = quaternion_from_euler(0.0, 0.0, math.radians(angle_deg))
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        return pose

    def _deproject(self, u: float, v: float, z: float):
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return float(x), float(y), float(z)

    def _transform_pose(
        self, pose: PoseStamped, target_frame: str
    ) -> Optional[PoseStamped]:
        if target_frame == pose.header.frame_id:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time.from_msg(pose.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    pose.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"Missing TF {target_frame} <- {pose.header.frame_id}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return None

        return apply_transform(pose, transform)

    def _draw_debug(self, image: np.ndarray, candidate: TissueCandidate):
        box = cv2.boxPoints(candidate.rect)
        box = np.intp(box)
        cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
        center = tuple(int(v) for v in candidate.rect[0])
        cv2.circle(image, center, 4, (0, 0, 255), -1)
        target = (
            int(candidate.rect[0][0] + self.target_center_offset_px[0]),
            int(candidate.rect[0][1] + self.target_center_offset_px[1]),
        )
        cv2.circle(image, target, 5, (255, 0, 0), -1)
        cv2.putText(
            image,
            "blue tissue z=%.3fm area=%.0f" % (candidate.depth_m, candidate.area),
            (max(0, center[0] - 80), max(20, center[1] - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    def _publish_tissue_tf(
        self, pose_camera: PoseStamped, pose_target: Optional[PoseStamped]
    ):
        if not self.tissue_frame_id:
            return

        tf_pose = pose_target
        if self.tissue_tf_parent_frame:
            if (
                tf_pose is None
                or tf_pose.header.frame_id != self.tissue_tf_parent_frame
            ):
                tf_pose = self._transform_pose(pose_camera, self.tissue_tf_parent_frame)
        if tf_pose is None:
            return

        transform = TransformStamped()
        transform.header.stamp = tf_pose.header.stamp
        transform.header.frame_id = tf_pose.header.frame_id
        transform.child_frame_id = self.tissue_frame_id
        transform.transform.translation.x = tf_pose.pose.position.x
        transform.transform.translation.y = tf_pose.pose.position.y
        transform.transform.translation.z = tf_pose.pose.position.z
        transform.transform.rotation = tf_pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_debug(self, image: np.ndarray, header):
        self._publish_image(self.debug_pub, image, header)

    def _publish_image(self, publisher, image: np.ndarray, header):
        msg = Image()
        msg.header = header
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = np.ascontiguousarray(image).tobytes()
        publisher.publish(msg)


def apply_transform(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    q_tf = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ]
    q_pose = [
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ]
    rot = rotation_matrix_from_quaternion(q_tf)
    point = np.array(
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
        dtype=np.float64,
    )
    translation = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ],
        dtype=np.float64,
    )
    transformed_point = rot.dot(point) + translation
    transformed_q = quaternion_multiply(q_tf, q_pose)

    out = PoseStamped()
    out.header.stamp = pose.header.stamp
    out.header.frame_id = transform.header.frame_id
    out.pose.position.x = float(transformed_point[0])
    out.pose.position.y = float(transformed_point[1])
    out.pose.position.z = float(transformed_point[2])
    out.pose.orientation.x = float(transformed_q[0])
    out.pose.orientation.y = float(transformed_q[1])
    out.pose.orientation.z = float(transformed_q[2])
    out.pose.orientation.w = float(transformed_q[3])
    return out


def main(args=None):
    rclpy.init(args=args)
    node = TissueDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
