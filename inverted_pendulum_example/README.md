# 用于验证plugin的倒立摆仿真环境
## 1. 可用功能
- 使用foxglove监看倒立摆的物理状态以及pid输出
- 使用foxglove调整单环pid参数

## 2. 运行步骤
- 安装gazebo harmonic， ROS2 jazzy， foxglove
- 编译插件并启动gazebo仿真环境
```bash
cd yourworkspace/inverted_pendulum_example
colcon build --packages-select controller_with_ros --symlink-install
source install/setup.bash
. launch.bash
```
- 启动foxglove studio，连接到ROS2桥接
- 监看/joint_states topic, [0]theta，[1]theta_dot
- 监看/controller_info, [0]now，[1]error, [2]out, [3]error_sum
- 使用foxglove的publish，向/pid_params topic发布新的pid参数，格式如下：
```json
{
  "layout": {
    "dim": [
      {
        "label": "",
        "size": 4,
        "stride": 0
      }
    ],
    "data_offset": 0
  },
  "data": [
    5.0,0.1,0.5,-0.2
  ]
}
```
- data 中依次为kp，ki，kd，目标位置target_theta
