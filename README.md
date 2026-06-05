<div align="center">

# SO-ARM101 视觉抓取项目

基于 ROS2 Humble 的机械臂视觉抓取流程

USB 摄像头识别红色物块，驱动 SO-ARM101 完成 sim 抓取和真实抓取。

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10.12-3776AB?logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.5.4-5C3EE8?logo=opencv&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-computing-013243?logo=numpy&logoColor=white)
![RViz2](https://img.shields.io/badge/RViz2-visualization-4B5563)
![License](https://img.shields.io/badge/license-Apache--2.0-0B76B7)

[硬件器件](#硬件器件) · [Pipeline](#pipeline) · [常用参数](#常用参数) · [注意事项](#注意事项)

</div>

## 硬件器件

- [SO-ARM101 / so_arm101 机械臂](https://github.com/TheRobotStudio/SO-ARM100)
- USB 摄像头
- 7.4V 锂电池
- [微雪舵机控制板](https://weixuesm.tmall.com/)
- [ST3215 舵机总线 Python 封装](https://pypi.org/project/st3215/)

## Pipeline

以下命令默认在仓库最外层目录运行：

```bash
cd ~/lerobot
```

### 1. 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lerobot
source install/setup.bash
```

如果重新打开了一个新终端，需要重新执行：

```bash
cd ~/lerobot
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2. 检查摄像头和舵机

```bash
python3 test/test_init.py
```

如果串口不是 `/dev/ttyACM0`，需要在相关 launch 命令里传入实际串口，例如：

```bash
ros2 launch lerobot so101_real_red_block_grasp.launch.py serial_port:=/dev/ttyUSB0
```

### 3. 标定工作区域

打开摄像头画面后，按顺序点击工作区域四个点：

1. 左上角
2. 右上角
3. 右下角
4. 左下角

然后按 `s` 保存，按 `q` 退出。

```bash
python3 -m lerobot.vision.detect_workspace
```

### 4. 生成平面透视变换

```bash
python3 -m lerobot.vision.plane_transformation
```

这个步骤会根据 `data/workspace_points.json` 生成：

```text
data/plane_transform.json
```

### 5. 单独测试红色物块识别

```bash
python3 -m lerobot.vision.detect_red_block
```

检测结果会持续写入：

```text
data/block_location.json
```

### 6. 只显示机械臂模型

不连接真实机械臂，只用 RViz 查看模型：

```bash
ros2 launch lerobot display_so101.launch.py use_hardware_joint_state_publisher:=false use_gui_joint_state_publisher:=true
```

连接真实机械臂，并从舵机读取 `/joint_states`：

```bash
ros2 launch lerobot display_so101.launch.py serial_port:=/dev/ttyACM0 use_hardware_joint_state_publisher:=true
```

### 7. 运行 sim 抓取演示

```bash
ros2 launch lerobot so101_sim_grasp_demo.launch.py
```

如果想让仿真动作慢一点，可以调大 `step_duration`：

```bash
ros2 launch lerobot so101_sim_grasp_demo.launch.py step_duration:=4.0
```

如果摄像头编号不是默认值 `4`，传入实际编号：

```bash
ros2 launch lerobot so101_sim_grasp_demo.launch.py camera_index:=0
```

### 8. 运行真实红块抓取

确认机械臂供电正常、舵机控制板串口正确、工作区域已经标定完成后运行：

```bash
ros2 launch lerobot so101_real_red_block_grasp.launch.py serial_port:=/dev/ttyACM0
```

如果夹爪抓物体时负载较大，建议降低速度和加速度：

```bash
ros2 launch lerobot so101_real_red_block_grasp.launch.py serial_port:=/dev/ttyACM0 speed:=50 acc:=3
```

如果摄像头编号不是默认值 `4`：

```bash
ros2 launch lerobot so101_real_red_block_grasp.launch.py serial_port:=/dev/ttyACM0 camera_index:=0
```

### 9. 查看抓取状态

另开一个终端：

```bash
cd ~/lerobot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /so101/grasp_state
```

常见状态包括：

```text
WAIT_FOR_BLOCK
MOVE_READY
PLAN_PRE_GRASP
MOVE_PRE_GRASP
OPEN_GRIPPER
DESCEND
CLOSE_GRIPPER
LIFT
MOVE_PLACE
RETURN_READY
RETURN_HOME
IDLE
ERROR
```

### 10. 手动触发一次真实抓取

如果启动时不想自动抓取，可以先关闭 `auto_pick`：

```bash
ros2 launch lerobot so101_real_red_block_grasp.launch.py serial_port:=/dev/ttyACM0 auto_pick:=false
```

另开一个终端手动触发：

```bash
cd ~/lerobot
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /so101/pick_red_block std_srvs/srv/Trigger "{}"
```

## 常用参数

- `camera_index`：USB 摄像头编号，默认 `4`
- `serial_port`：舵机控制板串口，默认 `/dev/ttyACM0`
- `speed`：真实舵机运动速度，数值越小越慢
- `acc`：真实舵机运动加速度，数值越小越柔和
- `step_duration`：sim 抓取每段插值时间，数值越大动画越慢
- `fixed_grasp_wrist_roll_rad`：抓取时固定的腕部旋转角度

## 注意事项

- 真实抓取前先确认 7.4V 锂电池供电稳定。
- 如果 `failed to move servo 6` 出现在夹住物体后，通常是夹爪负载过大，可以先降低 `speed` 和 `acc`。
- 如果摄像头打不开，先确认实际设备编号，再调整 `camera_index`。
- `data/*.json` 是本机标定和运行数据，不建议直接复用别的机器上的数据。
