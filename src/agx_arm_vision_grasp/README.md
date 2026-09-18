# agx_arm_vision_grasp

Eye-in-hand RealSense tissue-packet detection and grasping for AGX/Piper arms.

The package is intentionally split into two nodes:

- `tissue_detector`: subscribes to aligned RGB-D images, detects the blue rectangular tissue packet, and publishes a `PoseStamped`.
- `tissue_pick`: consumes the target pose and commands the existing AGX arm topics for a conservative open-loop grasp sequence.

Default runtime mode is `dry_run:=true`, so no arm or gripper command is sent until explicitly disabled.

## Topics

Detector input defaults:

- `camera/camera/color/image_raw`
- `camera/camera/aligned_depth_to_color/image_raw`
- `camera/camera/color/camera_info`

Detector output defaults:

- `vision/tissue_pose_camera`: target pose in the RealSense optical frame.
- `vision/tissue_pose`: target pose transformed to `base_link`.
- `vision/tissue_debug/image`: debug image with the selected rectangle.
- `vision/tissue_detected/image`: stable result image stream; frames are boxed when the blue tissue packet is detected.
- `/tf`: dynamic transform `base_link -> tissue_target` while the blue tissue packet is detected.

Pick node input/output defaults:

- subscribes `vision/tissue_pose`
- subscribes `feedback/tcp_pose`
- publishes `control/move_p`, `control/move_l`
- publishes `control/joint_states` for the `gripper` joint
- service `pick_tissue` (`std_srvs/srv/Trigger`)
- service `reset_home` (`std_srvs/srv/Trigger`)

## Start

Build and source:

```bash
cd ~/agx_arm_ws
colcon build --packages-select agx_arm_vision_grasp
source install/setup.bash
```

Start the arm with the gripper enabled:

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

Start RealSense with aligned depth enabled. Topic names must match `config/tissue_grasp.yaml`.

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

Start perception and grasp planning in dry-run mode:

```bash
ros2 launch agx_arm_vision_grasp tissue_eye_in_hand_grasp.launch.py dry_run:=true
```

The launch file includes the current eye-in-hand transform from `tcp_link` to the RealSense `camera_link`:

- translation: `[-0.07468078225321914, -0.011606012607028928, -0.09446864264218623]` m
- RPY rotation: `[-0.0321356983588658, -1.2015408589509187, 0.009178550859444086]` rad
- quaternion: `[-0.010659791335618892, -0.5652600727810003, -0.005297124935034859, 0.8248268663396712]`

That static transform is enabled by default. If you need to override it:

```bash
ros2 launch agx_arm_vision_grasp tissue_eye_in_hand_grasp.launch.py \
  dry_run:=true \
  enable_static_camera_tf:=true \
  camera_parent_frame:=tcp_link \
  camera_child_frame:=camera_link \
  camera_x:=0.0 camera_y:=0.0 camera_z:=0.0 \
  camera_roll:=0.0 camera_pitch:=0.0 camera_yaw:=0.0
```

Without a transform connecting the RealSense frame tree to the arm, the detector can still publish `vision/tissue_pose_camera`, but the pick node cannot command a base-frame grasp.

## Tune Detection

The default detector looks for the blue tissue packet shown in the RealSense view. Tune these in `config/tissue_grasp.yaml` only if the lighting or packaging changes:

- `hsv_h_min`, `hsv_h_max`: blue hue range. The current packet is around H `94-109` in OpenCV HSV.
- `hsv_s_min`, `hsv_v_min`: reject gray/white desk objects and dark shadows.
- `min_depth_m`, `max_depth_m`: valid working distance from the wrist camera.
- `min_area_px`: reject small false positives.
- `min_aspect_ratio`, `max_aspect_ratio`: reject non-rectangular objects.
- `min_blue_fraction`: reject loose blue fragments that do not fill enough of their fitted rectangle.
- `target_center_offset_px`: optional pixel offset from the detected blue rectangle center to the desired grasp point.

Check the debug image:

```bash
ros2 topic echo /vision/tissue_pose
```

Use `rqt_image_view` or RViz to inspect `vision/tissue_debug/image`.
Use `vision/tissue_detected/image` for a stable image stream; detected tissue packets are boxed.

Check the tissue TF:

```bash
ros2 run tf2_ros tf2_echo base_link tissue_target
```

## Trigger Grasp

After the target pose is stable and dry-run planned poses look reasonable:

```bash
ros2 launch agx_arm_vision_grasp tissue_eye_in_hand_grasp.launch.py dry_run:=false
ros2 service call /pick_tissue std_srvs/srv/Trigger {}
```

Manually return to the configured home TCP pose:

```bash
ros2 service call /reset_home std_srvs/srv/Trigger {}
```

The pick sequence is:

1. open gripper
2. move to pregrasp
3. approach target
4. close gripper
5. lift

The arm does not return home automatically after a pick by default. Use `/reset_home` when you want to return manually.

Tune these before using the real robot:

- `approach_axis_tcp`: TCP-frame approach direction. Default is `[0, 0, 1]`.
- `approach_distance_m`: distance before the target.
- `grasp_depth_offset_m`: additional motion past the visible surface.
- `lift_offset_base`: lift vector after closing.
- `return_home_after_pick`: optionally move back to `home_tcp_pose` after lifting. Default is `false`.
- `home_tcp_pose`: TCP home pose `[x, y, z, qx, qy, qz, qw]` in `base_link`.
- `abort_on_motion_timeout`: stop the sequence if pregrasp, grasp, lift, or home is not reached in time.
- `workspace_min`, `workspace_max`: workspace bounds safety check.
