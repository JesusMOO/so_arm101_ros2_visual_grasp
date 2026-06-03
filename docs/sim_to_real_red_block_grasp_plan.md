# SO101 Sim-to-Real 动态视觉识别抓取红色物块计划

## 目标

在现有 `lerobot` ROS2 包基础上，完成一套 SO101 机械臂动态视觉识别并抓取红色物块的 sim-to-real 流程。目标不是重写已有代码，而是把当前已经能工作的视觉标定、红块检测、URDF/RViz、MuJoCo 模型和 ST3215 舵机控制逐步接入 ROS2 节点。

最终运行效果：

1. 摄像头实时识别工作区内红色物块。
2. RViz2 中的 SO101 模型跟随真实舵机运动。
3. 控制节点根据红块位置规划并执行抓取流程。
4. 同一套上层抓取状态机可以切换真实机械臂或仿真后端。

## 当前已有资源

### 视觉侧

- `lerobot/vision/detect_workspace.py`
  - 已支持 USB 摄像头 `CAMERA_INDEX = 4`
  - 已使用 Linux `cv2.CAP_V4L2`
  - 已能保存工作区四点到 `data/workspace_points.json`

- `lerobot/vision/plane_transformation.py`
  - 已能读取 `workspace_points.json`
  - 已能计算透视变换矩阵
  - 已能保存 `data/plane_transform.json`

- `lerobot/vision/detect_red_block.py`
  - 已能基于 HSV 检测红色物块
  - 已能在透视变换后的工作区坐标中输出中心点、bbox、area
  - 已能保存 `data/block_location.json`

- `data/`
  - 已有 `workspace_points.json`
  - 已有 `plane_transform.json`
  - 已有 `block_location.json`

### 机器人模型侧

- `urdf/so101/so101_new_calib.urdf`
  - 已定义 SO101 的 6 个主要关节：
    - `shoulder_pan`
    - `shoulder_lift`
    - `elbow_flex`
    - `wrist_flex`
    - `wrist_roll`
    - `gripper`

- `rviz/so101.rviz`
  - 已配置 `RobotModel`
  - 固定坐标系为 `base_link`

- `launch/display_so101.launch.py`
  - 已启动 `robot_state_publisher`
  - 已启动 `joint_state_publisher`
  - 已启动 `rviz2`

### 仿真侧

- `mujoco/so101/so101_new_calib.xml`
- `mujoco/so101/so101_old_calib.xml`
- `mujoco/so101/scene.xml`
- `mujoco/so101/joints_properties.xml`

这些文件先作为 sim-to-real 的仿真模型基础，不在第一阶段重建模型。

### 舵机侧

- `test/test_init.py`
  - 已用于检查摄像头和舵机是否可连接。

- `test/test_servo_motion.py`
  - 已能通过 `ST3215.MoveTo(id, raw)` 手动移动单个舵机。

- `st3215` 包可用接口：
  - `ListServos()`
  - `ReadPosition(id)`
  - `MoveTo(id, raw, speed=..., acc=..., wait=...)`
  - `IsMoving(id)`
  - `ReadVoltage(id)`
  - `ReadTemperature(id)`
  - `ReadLoad(id)`

## 不重复工作的原则

1. 不重写红色检测算法，先复用 `detect_red_block.py` 中的 HSV、形态学处理、最大轮廓检测逻辑。
2. 不重写工作区标定，继续使用 `workspace_points.json` 和 `plane_transform.json`。
3. 不重复建模，继续使用现有 URDF、RViz、MuJoCo 文件。
4. 不修改 `install/` 和 `build/` 下的生成文件；所有后续改动只放在源码目录。
5. 不使用“预设姿态 + 图像坐标插值”作为抓取主方案；抓取目标必须通过相机外参、机械臂正运动学和逆运动学计算得到。

## 推荐总体方案

采用 ROS2-first 的分层方案：

```text
OpenCV 摄像头
    ↓
red_block_detector_node
    ↓
红块图像/工作区坐标 /red_block/center
    ↓
camera_to_base_transform
    ↓
红块在 base_link 下的三维位姿
    ↓
so101_kinematics
    ↓
IK 求解目标关节角
    ↓
grasp_controller_node
    ↓
SO101 目标关节角
    ↓
so101_driver_node
    ↓
ST3215 舵机
    ↓
so101_joint_state_publisher_node
    ↓
/joint_states
    ↓
robot_state_publisher
    ↓
RViz2 实时模型
```

仿真时替换底层后端：

```text
grasp_controller_node
    ↓
SimArmBackend / MuJoCoBackend
    ↓
/joint_states
    ↓
RViz2 或 MuJoCo
```

这样上层视觉和抓取状态机不关心当前是真实机械臂还是仿真机械臂。

## 方案对比

### 方案 A：ROS2-first + 严谨运动学抓取

优点：

- 目标位姿、关节角和 RViz/真实机械臂之间有明确数学关系。
- 后续接 MoveIt 或 MuJoCo 时不需要推翻上层接口。
- 可以用 FK/IK 单元测试验证结果，而不是依赖经验插值。

缺点：

- 前期必须完成相机到 `base_link` 的外参标定。
- 需要从 URDF 或模型文件中提取准确的关节轴、关节原点和连杆变换。
- IK 需要处理多解、不可达目标、关节限位和奇异位形。

推荐作为第一版。

### 方案 B：MoveIt 完整规划

优点：

- 后续扩展性更好。
- 可以做碰撞检查、路径规划、笛卡尔空间目标。

缺点：

- 当前仓库没有 MoveIt 配置包。
- 需要补 SRDF、planning group、kinematics.yaml、controller 配置。
- 初期调试成本高。

适合作为严谨运动学版本稳定后的增强项。第一版可以先实现轻量 FK/IK，再决定是否接 MoveIt。

### 方案 C：MuJoCo-first

优点：

- 可以先在仿真里调通抓取状态机。
- 避免真实机械臂初期碰撞风险。

缺点：

- MuJoCo 和 ROS2 的实时同步、关节命令映射需要额外封装。
- 如果视觉仍来自真实摄像头，仿真闭环意义有限。

适合作为并行验证工具，不建议作为第一条主线。

## 分阶段实施计划

### 阶段 0：整理运行入口和配置

目标：先把硬件、视觉、模型的关键配置集中，减少散落常量。

计划新增：

- `lerobot/config/so101_config.py`
  - 串口：`/dev/ttyACM0`
  - 摄像头：`CAMERA_INDEX = 4`
  - 舵机 ID：`1..6`
  - URDF 关节名列表
  - 舵机 raw 范围：`0..4095`
  - 默认速度、加速度

- `lerobot/common/paths.py`
  - 统一提供项目根目录和 `data/` 路径。

复用：

- 当前 `data/workspace_points.json`
- 当前 `data/plane_transform.json`

验收：

- 所有视觉脚本不再各自硬编码 `ROOT_DIR`。
- 后续节点统一从配置读取摄像头、串口、数据路径。

### 阶段 1：把红块检测改成 ROS2 节点

目标：保留已有 OpenCV 检测逻辑，把文件输出升级成 ROS2 topic 输出。

计划新增：

- `lerobot/vision/red_block_detector_node.py`

节点职责：

- 打开 `/dev/video4` 对应的 OpenCV 摄像头。
- 读取 `data/plane_transform.json`。
- 复用 `detect_red_block.py` 的：
  - `load_plane_transform`
  - `red_mask`
  - `detect_largest_red_block`
  - `draw_detection`
- 发布红块中心点。
- 继续保存 `data/block_location.json` 作为调试记录。

建议 topic：

- `/red_block/found`：`std_msgs/Bool`
- `/red_block/center`：`geometry_msgs/PointStamped`
- `/red_block/debug_marker`：`visualization_msgs/Marker`

坐标约定：

- 第一版发布的是透视变换后的工作区二维坐标：
  - `x`：0 到 `target_width`
  - `y`：0 到 `target_height`
  - `z`：0
- 后续再把该二维坐标映射成机械臂底座坐标系下的米制坐标。

验收：

```bash
ros2 run lerobot red_block_detector_node
ros2 topic echo /red_block/center
```

能在移动红色物块时看到 `/red_block/center` 连续变化。

### 阶段 2：让真实舵机状态驱动 RViz

目标：真实 SO101 一动，RViz2 模型同步动。

计划新增：

- `lerobot/so101/servo_bus.py`
- `lerobot/so101/joint_mapping.py`
- `lerobot/so101/joint_state_publisher_node.py`

`servo_bus.py` 职责：

- 封装 `ST3215`。
- 提供：
  - `connect()`
  - `close()`
  - `scan_servos()`
  - `read_position(servo_id)`
  - `move_to(servo_id, raw, speed, acc, wait)`

`joint_mapping.py` 职责：

- 维护舵机 ID 到 URDF 关节名的映射：

```python
SERVO_TO_JOINT = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
```

- 提供 `raw_to_rad(servo_id, raw)`。
- 提供 `rad_to_raw(servo_id, rad)`。
- 每个关节需要单独标定：
  - `raw_center`
  - `direction`
  - `rad_offset`
  - `lower_rad`
  - `upper_rad`

`joint_state_publisher_node.py` 职责：

- 以固定频率读取 6 个舵机当前位置。
- 发布 `sensor_msgs/JointState` 到 `/joint_states`。
- RViz2 通过 `robot_state_publisher` 自动更新模型。

需要修改：

- `launch/display_so101.launch.py`
  - 增加参数 `use_gui_joint_state_publisher`。
  - 默认关闭原来的 `joint_state_publisher`。
  - 启动新的 `so101_joint_state_publisher_node`。

验收：

```bash
ros2 launch lerobot display_so101.launch.py calibration:=new
ros2 topic echo /joint_states
```

手动移动真实机械臂或运行 `test/test_servo_motion.py` 时，RViz2 模型应同步变化。

### 阶段 3：建立安全的关节控制后端

目标：上层抓取控制不直接调用 `ST3215.MoveTo`，而是通过安全封装控制机械臂。

计划新增：

- `lerobot/so101/arm_backend.py`
- `lerobot/so101/real_arm_backend.py`
- `lerobot/so101/sim_arm_backend.py`

共同接口：

- `get_joint_positions() -> dict[str, float]`
- `move_joints(target: dict[str, float], speed: int, acc: int, wait: bool)`
- `open_gripper()`
- `close_gripper()`
- `stop()`

真实后端：

- 使用 `ST3215.MoveTo`。
- 使用 `rad_to_raw` 转换目标角度。
- 每次运动前检查 URDF 关节上下限。

仿真后端：

- 第一版只发布目标 `/joint_states`，用于 RViz 验证状态机。
- 第二版再接 MuJoCo 的 `qpos` 和 `ctrl`。

安全要求：

- 第一次只允许低速度、低加速度。
- 任意关节目标超出 URDF 限制时拒绝执行。
- 串口异常时停止状态机。
- 如果读取舵机失败，不继续抓取。

### 阶段 4：建立严谨运动学链路

目标：把红块检测结果转换成 `base_link` 坐标系下的三维抓取位姿，再通过 SO101 的 FK/IK 求解机械臂目标关节角。

这一阶段不使用图像坐标到关节角的经验插值。必须显式建立下面的数学链路：

```text
像素坐标 (u, v)
  ↓ 相机内参、深度/平面约束
相机坐标 p_camera
  ↓ 外参 T_base_camera
base_link 坐标 p_base
  ↓ 抓取姿态构造
末端目标位姿 T_base_ee
  ↓ SO101 IK
目标关节角 q_target
```

计划新增：

- `data/camera_calibration.json`
  - 相机内参矩阵 `K`
  - 畸变参数 `distortion`
  - 图像尺寸

- `data/camera_to_base_transform.json`
  - `parent_frame`: `base_link`
  - `child_frame`: `camera_color_optical_frame`
  - 平移 `translation_m`
  - 四元数 `rotation_xyzw`

- `lerobot/kinematics/so101_model.py`
  - 从 URDF 中维护 SO101 关节名、关节轴、关节原点、上下限。
  - 第一版可以手动解析 URDF 所需字段，后续可替换为 `urdfpy` 或 KDL。

- `lerobot/kinematics/so101_fk.py`
  - 输入 `dict[str, float]` 关节角。
  - 输出每个 link 到 `base_link` 的齐次变换矩阵。
  - 输出末端夹爪位姿 `T_base_ee`。

- `lerobot/kinematics/so101_ik.py`
  - 输入目标位姿 `T_base_ee` 和当前关节角。
  - 使用数值 IK 求解目标关节角。
  - 约束：
    - 关节上下限
    - 最大单步变化
    - 位置误差阈值
    - 姿态误差阈值
  - 不可达时返回明确失败，不发送舵机命令。

- `lerobot/vision/camera_geometry.py`
  - 加载相机内参和外参。
  - 将红块中心从像素/工作区坐标转换为 `base_link` 下三维点。
  - 第一版如果没有深度相机，就使用桌面平面约束：红块中心落在已知 `z = table_height_m` 平面。

- `lerobot/grasp/grasp_pose_planner.py`
  - 根据 `p_base` 构造抓取目标位姿：
    - 夹爪中心对准红块中心。
    - 夹爪 approach 方向垂直或近似垂直桌面。
    - 预抓取位姿在目标上方 `pre_grasp_height_m`。
    - 抓取位姿在红块高度附近。

验收：

- 给定一组关节角，FK 输出的末端位姿稳定可复现。
- 给定 FK 生成的末端位姿，IK 能求回接近原始关节角的解。
- 给定 `data/block_location.json` 中的红块中心，能输出 `base_link` 下的三维点。
- 给定一个可达抓取位姿，IK 输出不越界目标关节角。
- 给定不可达抓取位姿，IK 返回失败，控制节点不发送真实舵机命令。

### 阶段 5：抓取状态机

目标：把检测、接近、抓取、抬起、放置串成完整流程。

计划新增：

- `lerobot/grasp/red_block_grasp_controller_node.py`

状态机：

```text
IDLE
  ↓
WAIT_FOR_BLOCK
  ↓
PLAN_PRE_GRASP
  ↓
MOVE_PRE_GRASP
  ↓
OPEN_GRIPPER
  ↓
DESCEND
  ↓
CLOSE_GRIPPER
  ↓
LIFT
  ↓
MOVE_PLACE
  ↓
OPEN_GRIPPER
  ↓
RETURN_HOME
  ↓
IDLE
```

第一版抓取方式：

- 物块位置来自 `/red_block/center`。
- `camera_geometry.py` 将物块位置转换到 `base_link`。
- `grasp_pose_planner.py` 生成预抓取、抓取、抬起和放置的末端位姿。
- `so101_ik.py` 把每个末端目标位姿转换成目标关节角。
- 所有目标关节角必须通过 `arm_backend.py` 的限位检查后才能发送。

触发方式：

- `/so101/pick_red_block`：`std_srvs/Trigger`

调试 topic：

- `/so101/grasp_state`：`std_msgs/String`
- `/so101/grasp_target_marker`：`visualization_msgs/Marker`

验收：

```bash
ros2 service call /so101/pick_red_block std_srvs/srv/Trigger {}
ros2 topic echo /so101/grasp_state
```

能看到状态机按顺序推进。

### 阶段 6：整合 launch

目标：一条命令启动视觉、RViz、舵机状态发布和抓取控制。

计划新增：

- `launch/so101_red_block_grasp.launch.py`

包含节点：

- `robot_state_publisher`
- `rviz2`
- `red_block_detector_node`
- `so101_joint_state_publisher_node`
- `red_block_grasp_controller_node`

参数：

- `backend:=sim|real`
- `camera_index:=4`
- `serial_port:=/dev/ttyACM0`
- `calibration:=new`
- `start_rviz:=true`

启动命令：

```bash
colcon build --packages-select lerobot
source install/setup.bash
ros2 launch lerobot so101_red_block_grasp.launch.py backend:=sim
```

真实机械臂：

```bash
ros2 launch lerobot so101_red_block_grasp.launch.py backend:=real
```

### 阶段 7：MuJoCo 接入

目标：在不破坏真实机械臂后端的前提下，增加 MuJoCo 仿真后端。

计划新增：

- `lerobot/sim/mujoco_so101_backend.py`
- `launch/so101_mujoco_red_block_grasp.launch.py`

复用：

- `mujoco/so101/scene.xml`
- `mujoco/so101/so101_new_calib.xml`
- `mujoco/so101/joints_properties.xml`

第一版 MuJoCo 目标：

- 读取 SO101 MuJoCo 模型。
- 接收同一组目标关节角。
- 更新仿真关节位置。
- 发布 `/joint_states` 给 RViz2。

第二版 MuJoCo 目标：

- 加入红色物块几何体。
- 使用 MuJoCo 相机或外部 OpenCV 画面做仿真视觉。
- 比较仿真检测坐标和真实检测坐标。

注意：

- MuJoCo 不作为第一阶段阻塞项。
- 先用 `sim_arm_backend` 跑通 ROS2 状态机，再接 MuJoCo。

## 测试计划

### 单元测试

新增测试目录建议：

- `test/test_joint_mapping.py`
- `test/test_so101_forward_kinematics.py`
- `test/test_so101_inverse_kinematics.py`
- `test/test_camera_geometry.py`
- `test/test_grasp_pose_planner.py`
- `test/test_red_block_detection.py`
- `test/test_grasp_state_machine.py`

测试内容：

- `raw_to_rad` 和 `rad_to_raw` 互逆。
- 超出 URDF 限制时拒绝目标。
- `detect_largest_red_block` 能在合成图片中找到红色矩形。
- FK 对零位和若干已知关节角输出稳定的 link 位姿。
- IK 对 FK 生成的目标位姿能求回接近原始关节角的解。
- 相机几何能把红块中心点投影到 `base_link` 下的桌面平面。
- 抓取位姿规划器能生成预抓取、抓取、抬起和放置位姿。
- 状态机在 fake backend 下能从 `IDLE` 到 `RETURN_HOME`。

### 集成测试

不接硬件：

```bash
ros2 launch lerobot so101_red_block_grasp.launch.py backend:=sim
```

检查：

- RViz2 有模型。
- `/joint_states` 持续发布。
- `/red_block/center` 能发布。
- `/so101/grasp_state` 能变化。

接硬件：

```bash
python3 test/test_init.py
python3 test/test_servo_motion.py
ros2 launch lerobot so101_red_block_grasp.launch.py backend:=real
```

检查：

- 6 个舵机都能读取。
- RViz2 跟随真实舵机。
- 低速执行 home pose。
- 再执行单次抓取。

## 建议文件结构

```text
lerobot/
  common/
    paths.py
  config/
    so101_config.py
  vision/
    detect_workspace.py
    plane_transformation.py
    detect_red_block.py
    red_block_detector_node.py
    camera_geometry.py
  so101/
    servo_bus.py
    joint_mapping.py
    arm_backend.py
    real_arm_backend.py
    sim_arm_backend.py
    joint_state_publisher_node.py
  kinematics/
    so101_model.py
    so101_fk.py
    so101_ik.py
  grasp/
    grasp_pose_planner.py
    red_block_grasp_controller_node.py
  sim/
    mujoco_so101_backend.py
launch/
  display_so101.launch.py
  so101_red_block_grasp.launch.py
  so101_mujoco_red_block_grasp.launch.py
data/
  workspace_points.json
  plane_transform.json
  block_location.json
  camera_calibration.json
  camera_to_base_transform.json
docs/
  sim_to_real_red_block_grasp_plan.md
```

## 实施顺序

1. 先做 `so101_joint_state_publisher_node.py`，让 RViz2 跟随真实舵机。
2. 再做 `red_block_detector_node.py`，让红块检测从文件输出升级为 topic 输出。
3. 再做 `sim_arm_backend.py` 和 `real_arm_backend.py`，统一仿真和真实控制接口。
4. 再做 `so101_model.py`、`so101_fk.py`、`so101_ik.py`，建立可测试的运动学链路。
5. 再做 `camera_geometry.py`，完成相机坐标到 `base_link` 的外参变换。
6. 再做 `grasp_pose_planner.py` 和抓取状态机，在仿真后端中调通流程。
7. 最后接真实舵机和 MuJoCo。

这个顺序的原因：

- RViz 同步是后续所有调试的可视化基础。
- 视觉 topic 是抓取状态机的输入。
- FK/IK 是严谨抓取的核心，必须先通过单元测试再接状态机。
- 相机外参决定红块三维位置，必须在真实抓取前完成标定。
- sim 后端可以先验证状态机，不冒真实机械臂碰撞风险。
- real 后端最后接入，可以把风险控制在最小。

## 风险和注意事项

1. 舵机 raw 到 URDF rad 的映射必须单独标定，不能简单假设所有舵机方向一致。
2. `joint_state_publisher` GUI 节点和真实 `/joint_states` 发布节点不能同时发布同一组关节，否则 RViz 可能抖动。
3. 抓取前必须先限制速度和加速度。
4. 红块检测当前依赖颜色阈值，光照变化会影响稳定性。
5. 相机外参不准确会直接导致抓取点偏移，必须提供可复测的标定流程。
6. IK 可能出现多解、不可达或接近奇异位形，控制节点必须能拒绝失败结果。
7. `install/` 和 `build/` 是生成目录，后续只改源码后重新 `colcon build`。

## 最小可运行里程碑

### 里程碑 1：RViz 跟随真实机械臂

完成条件：

- `/joint_states` 来自真实 ST3215 读数。
- 移动真实 SO101 时，RViz2 模型同步动。

### 里程碑 2：红块检测 ROS2 化

完成条件：

- `/red_block/center` 能实时发布。
- `data/block_location.json` 仍保留为调试输出。

### 里程碑 3：严谨运动学链路

完成条件：

- FK 能根据关节角输出末端位姿。
- IK 能对 FK 生成的可达目标求回合法关节角。
- 红块中心能转换成 `base_link` 下三维点。

### 里程碑 4：仿真抓取状态机

完成条件：

- `backend:=sim` 下可以完整跑完抓取状态机。
- RViz2 中模型按状态机运动。
- 状态机使用 IK 输出的关节角，不使用预设姿态插值。

### 里程碑 5：真实低速抓取

完成条件：

- `backend:=real` 下低速完成一次红块抓取和放置。
- 异常时能停止，不继续发送危险动作。

### 里程碑 6：MuJoCo 验证

完成条件：

- MuJoCo 后端能接收同一组关节目标。
- RViz2 和 MuJoCo 的关节状态一致。

## 下一步建议

优先完成阶段 4 的运动学基础：先实现 FK，再用 FK 结果验证 IK。只有 FK/IK 和相机外参都可验证后，才进入抓取状态机。
