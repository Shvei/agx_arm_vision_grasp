# agx_arm_vision_grasp

Eye-in-hand（手眼一体）RealSense 纸巾识别与抓取，面向 AGX / Piper 机械臂。

本包订阅 RealSense 的对齐 RGB-D 图像，识别桌面上的蓝色纸巾包，输出目标位姿，
并规划一条「张开夹爪 → 到预抓取点 → 接近 → 闭合 → 抬起」的保守抓取序列。

---

## 1. 功能与数据流

```
RealSense D435（装在腕部，eye-in-hand）
  ├─ camera/camera/color/image_raw
  ├─ camera/camera/aligned_depth_to_color/image_raw
  └─ camera/camera/color/camera_info
                │  (ApproximateTimeSynchronizer 对齐)
                ▼
        ┌──────────────────┐
        │  tissue_detector │   HSV 蓝色分割 + 深度门限 + 形状筛选
        └──────────────────┘
                │
                ├─ vision/tissue_pose            (base_link，给抓取用)
                ├─ vision/tissue_pose_camera     (相机光学系，调试用)
                ├─ /tf  base_link → tissue_target
                └─ vision/tissue_detected/image  (带绿框的结果图)
                │
                ▼
        ┌──────────────────┐
        │   tissue_pick    │   由目标位姿算出 pregrasp / grasp / lift 三点
        └──────────────────┘
                │
                ├─ vision/pregrasp_pose, vision/grasp_pose, vision/lift_pose
                ├─ vision/grasp_plan (+ grasp_plan_markers)
                ├─ control/ik_pose        (PoseStamped)
                └─ control/joint_states   (夹爪开合)
                │
                ▼
        ┌──────────────────┐
        │ arm_ik_pose_node │   位姿 → 关节角（Pinocchio + CasADi/IPOPT）
        │ (agx_arm_ctrl 包)│
        └──────────────────┘
                │
                ▼
        control/move_j  (JointState)
                │
                ▼
   agx_arm_ctrl_single_node  →  CAN  →  Piper 机械臂
```

> **注意**：`tissue_pick` 的 `move_p_topic` 在配置里被指向了 `control/ik_pose`，
> 也就是走到了上面的 IK 节点，而不是控制器自带的 `move_p`。
> 参数名保持 `move_p_topic` 只是为了兼容，实际语义是「位姿指令出口」。

---

## 2. 目录结构

```
agx_arm_vision_grasp/
├── agx_arm_vision_grasp/
│   ├── geometry.py               # 四元数 / 旋转矩阵工具
│   ├── tissue_detector_node.py   # 蓝色纸巾检测
│   └── tissue_pick_node.py       # 抓取规划与执行
├── config/
│   └── tissue_grasp.yaml         # 两个节点的全部参数（唯一调参入口）
├── launch/
│   └── tissue_eye_in_hand_grasp.launch.py
├── resource/
├── package.xml
├── setup.py
└── README.md
```

---

## 3. 依赖

**ROS 2（Humble）**

```
rclpy  sensor_msgs  geometry_msgs  agx_arm_msgs  std_srvs
visualization_msgs  tf2_ros  image_transport  launch  launch_ros
```

**Python**

- `opencv-python`（`tissue_detector` 必需）
- `numpy`
- `pinocchio` + `casadi`（只有 `arm_ik_pose_node` 需要，属于 `agx_arm_ctrl` 包）

> **重要**：`pinocchio` / `eigenpy` 若是针对 **NumPy 2.x** 编译的，
> 运行时 NumPy 必须是 2.x，否则所有接受 Eigen 参数的 pinocchio 函数
> 都会抛 `Boost.Python.ArgumentError: ... did not match C++ signature`。
> 见第 12.1 节。

---

## 4. 编译

```bash
cd ~/agx_arm_ros
colcon build --packages-select agx_arm_vision_grasp agx_arm_ctrl agx_arm_msgs
source install/setup.bash
```

---

## 5. 快速开始

需要 **3 个终端**。

**① 启动机械臂（含夹爪）**

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper \
  auto_enable:=true tcp_offset:='[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]'
```

**② 启动 RealSense（必须是「对齐到彩色」的深度）**

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

**③ 启动识别与抓取**

```bash
# 先干跑，只规划不发指令（默认就是 dry_run:=true）
ros2 launch agx_arm_vision_grasp tissue_eye_in_hand_grasp.launch.py dry_run:=true

# 确认规划正常后再实际抓取
ros2 launch agx_arm_vision_grasp tissue_eye_in_hand_grasp.launch.py dry_run:=false
```

**触发抓取**

```bash
# 建议先回 home，让纸巾在视野内、检测最稳
ros2 service call /reset_home std_srvs/srv/Trigger {}

# 触发一次抓取
ros2 service call /pick_tissue std_srvs/srv/Trigger {}
```

**抓取序列**：张开夹爪 → 到 pregrasp → 到 grasp → 闭合夹爪 → 抬升到 lift。
默认**不会**自动回 home（`return_home_after_pick: false`），需要时手动调 `/reset_home`。

---

## 6. Launch 参数

`tissue_eye_in_hand_grasp.launch.py`：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `config` | `config/tissue_grasp.yaml` | 检测与抓取参数文件 |
| `dry_run` | `true` | `true` 时只规划、不发任何机械臂/夹爪指令 |
| `auto_pick` | `false` | `true` 时检测到目标自动抓一次 |
| `enable_ik_pose` | `true` | 是否一并启动 `arm_ik_pose_node` |
| `ik_config` | `agx_arm_ctrl/config/arm_ik_pose.yaml` | IK 节点参数文件 |
| `ik_python_executable` | conda 环境 python | 装有 pinocchio + casadi 的 python |
| `namespace` | `""` | 可选命名空间 |
| `enable_static_camera_tf` | `true` | 是否发布手眼静态 TF |
| `camera_parent_frame` | `tcp_link` | 静态 TF 父坐标系 |
| `camera_child_frame` | `camera_link` | 静态 TF 子坐标系 |
| `camera_x/y/z` | `-0.07468 / -0.01161 / -0.09447` | 手眼平移（米） |
| `camera_roll/pitch/yaw` | `-0.03214 / -1.20154 / 0.00918` | 手眼姿态（弧度） |

手眼标定值可以用启动参数覆盖：

```bash
ros2 launch agx_arm_vision_grasp tissue_eye_in_hand_grasp.launch.py \
  dry_run:=true \
  camera_parent_frame:=tcp_link camera_child_frame:=camera_link \
  camera_x:=0.0 camera_y:=0.0 camera_z:=0.0 \
  camera_roll:=0.0 camera_pitch:=0.0 camera_yaw:=0.0
```

> 静态 TF 必须把相机坐标系挂到机械臂 TF 树上（默认父系 `tcp_link`）。
> 如果 TF 断链，检测器只能发布 `vision/tissue_pose_camera`，
> `vision/tissue_pose` 不会生成，抓取也就无法规划。

---

## 7. 话题与服务

### 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `camera/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图 |
| `camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 对齐到彩色的深度 |
| `camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 内参 |
| `vision/tissue_pose` | `geometry_msgs/PoseStamped` | （tissue_pick）目标位姿 |
| `feedback/tcp_pose` | `geometry_msgs/PoseStamped` | （tissue_pick）当前 TCP 位姿 |
| `feedback/arm_status` | `agx_arm_msgs/AgxArmStatus` | （tissue_pick）机械臂状态 |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `vision/tissue_pose` | `PoseStamped` | 目标位姿，`base_link` |
| `vision/tissue_pose_camera` | `PoseStamped` | 目标位姿，相机光学系 |
| `vision/tissue_debug/image` | `Image` | 调试图 |
| `vision/tissue_detected/image` | `Image` | 结果图（检测到时画绿框） |
| `/tf` | `TFMessage` | `base_link → tissue_target` |
| `vision/pregrasp_pose` / `grasp_pose` / `lift_pose` | `PoseStamped` | 规划出的三个点 |
| `vision/grasp_plan` | `PoseArray` | 三点合集 |
| `vision/grasp_plan_markers` | `MarkerArray` | RViz 可视化（球 + 标签 + 折线） |
| `control/ik_pose` | `PoseStamped` | 发给 IK 节点的位姿指令 |
| `control/joint_states` | `JointState` | 夹爪开合指令（`name=["gripper"]`） |

### 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `pick_tissue` | `std_srvs/Trigger` | 执行一次抓取 |
| `reset_home` | `std_srvs/Trigger` | 回到 `home_tcp_pose` |

---

## 8. 抓取几何（核心）

所有位姿都在 `base_link` 下。沿**接近轴**（approach axis）看，三个点的先后关系是：

```
                          目标表面                     grasp
                    (+object_to_grasp_offset_base)      │
   pregrasp                 │                          │
      ●─────────────────────●──────────────────────────●
      │◀── approach_distance_m ──────────────────────▶│
                            │◀─ grasp_depth_offset_m ─▶│

   接近轴（夹爪 +Z 在基座系下的方向）──────────────────▶
```

计算式（见 `tissue_pick_node.py::_build_grasp_plan`）：

```python
approach_axis_base = R(orientation) @ approach_axis_tcp     # approach_mode == tcp_axis

grasp_p    = target_p + object_to_grasp_offset_base + approach_axis_base * grasp_depth_offset_m
             # 然后夹紧下限：grasp_p.z >= min_grasp_z_m

pregrasp_p = grasp_p - approach_axis_base * approach_distance_m
             # 然后夹紧下限：pregrasp_p.z >= min_pregrasp_z_m

lift_p     = grasp_p + lift_offset_base
```

三个点在发布前都要通过 `workspace_min` / `workspace_max` 检查，越界直接中止。

### 三个偏移量的区别（最容易混淆）

| 参数 | 参考系 | 含义 |
|---|---|---|
| `object_to_grasp_offset_base` | **基座系**（世界轴，固定） | 从目标点的刚性偏移。`[0,0,-0.015]` = 永远竖直向下 1.5 cm |
| `grasp_depth_offset_m` | **接近轴**（随夹爪姿态旋转） | 沿夹爪正前方「再扎进去」多少米 |
| `approach_distance_m` | **接近轴** | pregrasp 相对 grasp 的后退量（决定路径长度） |

> ⚠️ `grasp_depth_offset_m` 是**整体平移**，不是加长行程 ——
> 因为 `pregrasp` 由 `grasp` 反推，所以 pregrasp/grasp/lift 会一起平移，
> 从 pregrasp 走到 grasp 的距离永远还是 `approach_distance_m`。
>
> ⚠️ 它沿接近轴，所以同一个数值在不同 `orientation_mode` 下效果完全不同：
> 水平接近时它主要改变 X，改成垂直下压后它主要改变 Z（有压到桌面的风险）。

### 为什么需要 `grasp_depth_offset_m`

整条流水线把 **TCP**（`link6 + tool_translation_xyz`）当作夹持点。
如果这个长度量的是**指尖**而不是**两指中间**，TCP 到位时手指还差一截，
表现出「对准了纸巾、在正前方闭合、差一点夹到」。

此时有两种修法：

1. **补偿法**：保持 TCP 定义不变，把 `grasp_depth_offset_m` 加大一点；
2. **根治**：量出真实的「法兰 → 两指中间」距离 `L`，把 **两处** 同时设成 `L`：
   - `agx_arm_ctrl/launch/*.launch.py` 的 `tcp_offset`
   - `agx_arm_ctrl/config/arm_ik_pose.yaml` 的 `tool_translation_xyz`

> ⚠️ 这两处定义的是**不同的东西**，必须分清：
> - 手臂 launch 的 `tcp_offset` → 只影响 `feedback/tcp_pose` 反馈
>   以及控制器自带 `move_p`/`move_l` 的换算；
> - IK 的 `tool_translation_xyz` → **决定机械臂实际去哪**。
>
> 本包的抓取链路走 `control/ik_pose → IK → control/move_j`，而 `move_j` 是纯关节角控制，
> **改 launch 的 `tcp_offset` 不会改变机械臂的实际落点**。要改变落点请改
> `tool_translation_xyz` 或 `grasp_depth_offset_m`。

---

## 9. 参数详解

参数文件：`config/tissue_grasp.yaml`，分 `tissue_detector` 与 `tissue_pick` 两段。

### 9.1 tissue_detector

**话题 / 坐标系**

| 参数 | 默认 | 说明 |
|---|---|---|
| `color_topic` | `camera/camera/color/image_raw` | 彩色图话题 |
| `depth_topic` | `camera/camera/aligned_depth_to_color/image_raw` | 深度图话题 |
| `camera_info_topic` | `camera/camera/color/camera_info` | 内参话题 |
| `target_frame` | `base_link` | 目标位姿输出坐标系 |
| `camera_frame` | `camera_color_optical_frame` | 图像 header 无 frame_id 时的回退坐标系 |
| `tissue_frame_id` | `tissue_target` | 发布到 TF 的 frame 名 |
| `tissue_tf_parent_frame` | `base_link` | 上述 TF 的父系 |
| `pose_topic` | `vision/tissue_pose` | 目标位姿输出话题 |
| `pose_camera_topic` | `vision/tissue_pose_camera` | 相机系目标位姿输出话题 |
| `debug_image_topic` | `vision/tissue_debug/image` | 调试图话题 |
| `detected_image_topic` | `vision/tissue_detected/image` | 结果图话题 |
| `tf_timeout_sec` | `0.08` | TF 查询超时（秒） |

**深度**

| 参数 | 默认 | 说明 |
|---|---|---|
| `depth_scale` | `0.001` | 16UC1 深度单位→米（RealSense 通常 0.001） |
| `min_depth_m` / `max_depth_m` | `0.12` / `0.85` | 有效工作距离窗口 |

**颜色分割（OpenCV HSV 范围：H 0–179，S/V 0–255）**

| 参数 | 默认 | 说明 |
|---|---|---|
| `hsv_h_min` / `hsv_h_max` | `90` / `115` | 蓝色色相范围 |
| `hsv_s_min` / `hsv_s_max` | `60` / `255` | 饱和度下限用于排除灰白 |
| `hsv_v_min` / `hsv_v_max` | `60` / `255` | 亮度下限用于排除阴影 |
| `morphology_kernel_px` | `7` | 开/闭运算核大小 |

**形状筛选**

| 参数 | 默认 | 说明 |
|---|---|---|
| `min_area_px` | `2500.0` | 最小面积，滤小碎块 |
| `max_area_fraction` | `0.65` | 最大面积占全图比例 |
| `min_aspect_ratio` / `max_aspect_ratio` | `0.55` / `4.0` | 长宽比范围 |
| `min_rectangularity` | `0.55` | 面积 / 最小外接矩形面积 |
| `min_valid_depth_ratio` | `0.35` | 轮廓内有效深度像素占比下限 |
| `min_blue_fraction` | `0.45` | 见「已知限制」 |
| `target_center_offset_px` | `[0.0, 0.0]` | 检测中心到抓取点的像素偏移 |

**发布开关**：`publish_tissue_tf` / `publish_debug_image` / `publish_detected_image`。

### 9.2 tissue_pick

**话题**

| 参数 | 默认（配置值） | 说明 |
|---|---|---|
| `target_pose_topic` | `vision/tissue_pose` | 目标位姿输入 |
| `tcp_pose_topic` | `feedback/tcp_pose` | 当前 TCP 位姿 |
| `arm_status_topic` | `feedback/arm_status` | 机械臂状态 |
| `move_p_topic` | **`control/ik_pose`** | 位姿指令出口（指向 IK 节点） |
| `move_l_topic` | `control/move_l` | 仅 `use_linear_approach:=true` 时使用 |
| `gripper_topic` | `control/joint_states` | 夹爪指令（`name=["gripper"]`） |
| `pregrasp_pose_topic` | `vision/pregrasp_pose` | 预抓取点输出 |
| `grasp_pose_topic` | `vision/grasp_pose` | 抓取点输出 |
| `lift_pose_topic` | `vision/lift_pose` | 抬升点输出 |
| `plan_pose_array_topic` | `vision/grasp_plan` | 三点合集输出 |
| `plan_marker_topic` | `vision/grasp_plan_markers` | RViz MarkerArray 输出 |

**运行模式**

| 参数 | 默认 | 说明 |
|---|---|---|
| `dry_run` | `true` | 只规划不执行 |
| `auto_pick` | `false` | 检测到目标自动抓一次 |
| `return_home_after_pick` | `false` | 抓完是否自动回 home |
| `publish_plan_preview` | `true` | 周期性发布规划预览（5 Hz） |
| `plan_preview_period_sec` | `0.2` | 预览周期 |
| `use_linear_approach` | `false` | `true` 时 grasp/lift 走 `control/move_l` 直线运动 |
| `use_feedback_wait` | `true` | 是否用 TCP 反馈等待到位 |

**姿态与接近**

| 参数 | 配置值 | 说明 |
|---|---|---|
| `orientation_mode` | `current` | `current`=保持当前 TCP 姿态；`top_down_current_yaw`=竖直下压并保持当前偏航；`fixed`=用 `fixed_orientation_xyzw` |
| `fixed_orientation_xyzw` | `[1,0,0,0]` | `fixed` 模式使用 |
| `keep_current_orientation` | `true` | `current` 模式的行为开关 |
| `approach_mode` | `tcp_axis` | `tcp_axis`=沿夹爪正前方；`base_axis`=沿 `approach_axis_base` |
| `approach_axis_tcp` | `[0,0,1]` | TCP 系下的接近方向 |
| `approach_axis_base` | `[0,0,-1]` | `base_axis` 模式使用 |

**抓取几何**

| 参数 | 配置值 | 说明 |
|---|---|---|
| `approach_distance_m` | `0.10` | pregrasp 到 grasp 的行程 |
| `grasp_depth_offset_m` | `0.015` | 沿接近轴额外深入的量（夹持点补偿） |
| `object_to_grasp_offset_base` | `[0,0,-0.015]` | 基座系固定偏移，向下 1.5 cm |
| `min_grasp_z_m` | `0.01` | grasp 的 Z 下限（**只做桌面净空**，不是抓取高度） |
| `min_pregrasp_z_m` | `0.05` | pregrasp 的 Z 下限 |
| `lift_offset_base` | `[0,0,0.08]` | 抬升向量 |
| `workspace_min` / `workspace_max` | `[-0.65,-0.45,0.01]` / `[0.65,0.45,0.60]` | 工作空间包围盒 |

**夹爪**

| 参数 | 配置值 | 说明 |
|---|---|---|
| `open_width_m` | `0.085` | 张开宽度（米，AGX 夹爪量程 0–0.1） |
| `close_width_m` | `0.018` | 闭合宽度（米） |
| `gripper_effort_n` | `1.5` | 夹持力（N，AGX 夹爪量程 0.5–3.0） |

**时序与安全**

| 参数 | 配置值 | 说明 |
|---|---|---|
| `motion_timeout_sec` | `8.0` | 单段运动的等待超时 |
| `settle_time_sec` | `1.5` | 超时后的额外稳定时间 |
| `pose_tolerance_m` | `0.025` | 认为「到位」的位置容差 |
| `require_recent_target_sec` | `1.0` | 目标必须是这么新，否则拒绝触发 |
| `abort_on_motion_timeout` | `true` | 超时是否中止 |
| `abort_on_arm_error` | `true` | 机械臂报错是否中止 |

**安全边界（在 `agx_arm_ctrl/config/arm_ik_pose.yaml`）**

| 参数 | 说明 |
|---|---|
| `max_position_error_m` | IK 解的位置误差上限，默认 `0.03` |
| `max_orientation_error_rad` | IK 解的姿态误差上限，默认 `0.6` |
| `max_joint_step_rad` | IK 解相对当前关节角的最大单关节增量 |
| `require_feedback` | 没有关节反馈时是否拒绝解算，默认 `true` |

> `max_joint_step_rad` 是拿「IK 解」和「当前关节角」逐关节比。
> 本包下发的是**一次性绝对位姿**，第一次朝目标走的差值本来就可能很大
> （实测 0.82–1.3 rad），所以这个值不能设得太小（例如 0.8 会直接卡死机械臂）。
> 真正的安全边界是 IK 里的关节限位约束 + 可选的自碰撞检查。

---

## 10. 调试与可视化

**看图**

```bash
ros2 run rqt_image_view rqt_image_view        # 选 vision/tissue_detected/image
```

`vision/tissue_detected/image` 是稳定流，检测到时画绿框并标注
`blue tissue z=… area=…`（z 为相机系深度）；`vision/tissue_debug/image` 同样内容。

**看目标与规划**

```bash
ros2 topic echo --once /vision/tissue_pose          # base_link 下的目标
ros2 topic echo --once /vision/tissue_pose_camera   # 相机系下的目标
ros2 topic echo --once /vision/grasp_pose           # 规划出的抓取点
ros2 topic echo --once /feedback/tcp_pose           # 当前 TCP
ros2 topic hz /camera/camera/color/image_raw        # 相机是否在出图
```

**RViz**：加上 `MarkerArray` 显示 `vision/grasp_plan_markers`，
绿=pregrasp、红=grasp、蓝=lift，可直接目视三点是否合理。

**查 TF**

```bash
ros2 run tf2_ros tf2_echo base_link tissue_target
ros2 run tf2_ros tf2_echo tcp_link camera_link
```

在 RViz 里确认 `base_link` 到 `camera_color_optical_frame` 是**同一条 TF 树**。

---

## 11. 调参指南

| 现象 | 调整 |
|---|---|
| 检测不到纸巾 | 调 `hsv_h_min/max`（用 `rqt_image_view` 看调试图）；降 `min_area_px`；放宽 `min_depth_m`/`max_depth_m` |
| 误检到别的蓝色物体 | 升 `min_area_px`、`min_rectangularity`；收紧 `hsv_s_min`/`hsv_v_min` |
| 夹爪在纸巾**正前方/上方**闭合、差一点 | **加大 `grasp_depth_offset_m`** |
| 夹爪把纸巾**推走 / 压到桌面** | **减小 `grasp_depth_offset_m`** |
| 夹爪闭合后宽度到 `close_width_m`、力≈0 | 夹空了 → 加大 `grasp_depth_offset_m`；或检查目标是否偏移 |
| 夹爪闭合后宽度停在 `close_width_m` 以上、力≈`gripper_effort_n` | 夹住了 ✅ |
| 抓取点太高（悬空闭合） | 检查 `min_grasp_z_m` 是否把点抬起来了；看日志里的 `Raised grasp z from … to …` 警告 |
| 目标落在纸巾侧面而不是正面 | 试 `orientation_mode: top_down_current_yaw`（垂直下压更适合平放物体） |
| 从当前姿态不可达 | 调 `min_pregrasp_z_m`、`workspace_*`；或先 `/reset_home` 再触发 |
| 到位判断老超时 | 放宽 `pose_tolerance_m`，或加大 `motion_timeout_sec` |

**收敛思路**：一次只改一个量。
先 `dry_run:=true` 看 `vision/pregrasp_pose` / `grasp_pose` 是否合理（RViz 三点），
再 `dry_run:=false` 实测；深度相关的量以 **5 mm** 为步长微调。

---

## 12. 故障排查

### 12.1 `Boost.Python.ArgumentError: … did not match C++ signature`

pinocchio 报 `framesForwardKinematics(Model, Data, numpy.ndarray)` 类型不匹配。

**原因**：`eigenpy` / `pinocchio` 是针对 NumPy 2.x 编译的，而环境里生效的是 NumPy 1.x。
NumPy 2 编译的 C 扩展无法在 NumPy 1 运行时下工作，于是 eigenpy 静默地没有注册
numpy↔Eigen 转换器，**所有**接受 Eigen 参数的 pinocchio 函数都会报这个错。

**排查**

```bash
python -c "import numpy, pinocchio, eigenpy; print(numpy.__version__, pinocchio.__version__, eigenpy.__version__)"
python -c "
import numpy as np, pinocchio as pin
m = pin.buildModelFromUrdf('<你的 urdf>'); d = m.createData()
pin.framesForwardKinematics(m, d, np.zeros(m.nq)); print('FK OK')"
```

**修复**：让 numpy 与 pinocchio 的构建版本一致（通常是升到 2.x）。
注意环境里可能存在多份 numpy 残留，检查 `site-packages/numpy-*.dist-info`。

### 12.2 `Reject IK solution: max joint step X > Y`

`arm_ik_pose.yaml` 的 `max_joint_step_rad` 太小。这是**绝对位姿下发**接口，
首次朝目标走的差值天然较大。适当放宽（例如 3.2），
安全由 IK 的关节限位与碰撞检查负责。

### 12.3 `Raised grasp z from … to min_grasp_z_m …` 且夹爪悬空闭合

`min_grasp_z_m` 被当成了抓取高度。检测器给的是纸巾**顶面**，
应让 `min_grasp_z_m` 只承担桌面净空职责，抓取高度用
`object_to_grasp_offset_base` 和 `grasp_depth_offset_m` 表达。

### 12.4 夹爪完全不动（宽度恒定、力≈0）

先排除**硬件**：夹爪线是否插好、供电是否正常。
驱动在断线时仍可能报 `driver_enable_status=True` 并保持 170 Hz 通信，
唯一破绽就是宽度从不变化。

```bash
ros2 topic echo /feedback/gripper_status     # 看 width / force / driver_enable_status / homing_status
```

也可以直接绕过上层验证通路：

```bash
ros2 topic pub --once /control/joint_states sensor_msgs/msg/JointState \
  "{name: ['gripper'], position: [0.03], effort: [1.5]}"
```

若宽度随之变化，说明驱动正常，问题在发布侧。

### 12.5 触发被拒 `no recent tissue target pose`

触发时那一刻检测器没有输出（纸巾不在视野内，或从上一个抓取姿态触发）。
先 `/reset_home` 让纸巾回到视野，再触发；
必要时放宽 `require_recent_target_sec`。

### 12.6 触发被拒 `arm_status=N; call /clear_error before picking`

机械臂处于非 0 状态。常见 `arm_status=5`（`JOINT_COMMUNICATION_ERR`），
多为线束接触不良导致的瞬时抖动。

```bash
ros2 service call /clear_error std_srvs/srv/Trigger {}
```

若频繁复发，检查 CAN 线束与夹爪线。

### 12.7 `Timed out waiting for TCP feedback to reach target`

TCP 反馈在 `motion_timeout_sec` 内没进入 `pose_tolerance_m`。依次检查：

1. IK 是否接受了目标（看 `arm_ik_pose_node` 日志有没有 `Published move_j`）；
2. `feedback/tcp_pose` 与 IK 的 `tool_translation_xyz` 是否定义一致；
3. 到位容差与超时是否过紧。

### 12.8 目标是静止物体，但报出来的位置会漂

手眼标定（`camera_x/y/z` + `camera_roll/pitch/yaw`）精度不足。
相机在 30 cm 距离上，**1° 的姿态误差就是约 5 mm** 的位置误差，
且随视角变化 —— 表现为同一静止物体在不同手臂姿态下坐标不同。

**验证方法**：把纸巾固定在桌上不动，把手臂摆到 2–3 个不同姿态分别执行

```bash
ros2 topic echo --once /vision/tissue_pose
```

若坐标差出几厘米，就需要重新标定手眼变换。
在标定前，**每次都在同一姿态（先 `/reset_home`）触发抓取**，
可以把标定误差变成可复现的系统偏差，再用 `grasp_depth_offset_m` 稳定补偿。

---

## 13. 已知限制

- **单目标**：每个时刻只跟踪一个最优候选（按面积 × 矩形度 × 有效深度比打分），
  不支持多目标选择或跟踪。
- **开环抓取**：规划在触发时算一次，之后按序执行，不闭环修正位姿；
  目标在抓取过程中移动会导致失败。
- **检测抖动**：深度估计偶尔会出现离群值（实测有少量样本 z 明显偏大甚至为负）。
  当前实现取触发瞬间的最新一帧目标，**没有做多帧一致性校验**。
  若遇到间歇性抓空，可考虑加入多帧中值滤波 + 合理域（z 范围）拒绝。
- **`min_blue_fraction` 语义**：代码中该判断实际用的是 `rectangularity`
  （与上一条判断重复），并非真实的「轮廓内蓝色像素占比」。
  由于掩膜本身已按颜色生成，功能上影响有限，但参数名与语义不符。
- **手眼标定**为手测静态 TF，精度有限，见 12.8。
- **夹爪量程**：AGX 夹爪宽度 0–0.1 m、力 0.5–3.0 N，
  参数超出范围会被驱动拒绝。

---

## 14. 许可与致谢

本包基于 [agilexrobotics/agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros)（MIT）二次开发。
