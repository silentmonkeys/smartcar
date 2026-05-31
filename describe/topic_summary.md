# 当前拉起的 ROS 2 Topic 功能说明

本文根据当前系统 `ros2 topic list` 的输出整理，目的是帮助后续二次开发快速定位每个 topic 的作用、来源模块和可扩展方向。

## 1. 总览

当前启动的功能可以分成 6 类：

1. 底盘运动与里程计
2. 摄像头图像与点云
3. IMU 与电源状态
4. 机械臂/底盘状态反馈
5. 交互控制与灯光/蜂鸣器
6. TF 与导航相关接口

## 2. Topic 明细

| Topic                     | 作用                                      | 典型来源                                          | 二开关注点                                       |
| ------------------------- | ----------------------------------------- | ------------------------------------------------- | ------------------------------------------------ |
| /cmd_vel                  | 底盘速度指令，通常由导航、键盘或遥控发布  | 导航节点、手动控制节点                            | 二开最常用控制口，新的控制算法一般最终都汇入这里 |
| /odom                     | 底盘里程计，表示机器人位姿估计            | 底盘驱动、融合定位节点                            | 做 SLAM、导航、闭环控制时常用                    |
| /odom_raw                 | 原始里程计数据，未融合或未滤波            | 底盘驱动                                          | 适合调试轮速、编码器和底盘运动学                 |
| /scan                     | 激光雷达二维扫描数据                      | 雷达驱动                                          | SLAM、避障、建图、导航核心输入                   |
| /point_cloud              | 点云数据，通常由深度相机或雷达生成        | 点云处理节点、深度相机                            | 可用于三维避障、障碍物检测、环境建模             |
| /camera/color/image_raw   | 彩色图像原始流                            | Astra 相机                                        | 视觉识别、目标检测、视觉里程计等                 |
| /camera/color/camera_info | 彩色相机内参                              | Astra 相机                                        | 图像标定、畸变校正、三维重建                     |
| /camera/depth/image_raw   | 深度图原始流                              | Astra 相机                                        | 深度测距、避障、三维感知                         |
| /camera/depth/camera_info | 深度相机内参                              | Astra 相机                                        | 深度配准、三维点云生成                           |
| /camera/depth/points      | 深度点云                                  | Astra 相机或深度处理节点                          | 适合做点云滤波、障碍物识别                       |
| /camera/ir/image_raw      | 红外图像原始流                            | Astra 相机                                        | 视需要做近距离感知或调试                         |
| /camera/ir/camera_info    | 红外相机内参                              | Astra 相机                                        | 红外成像相关标定                                 |
| /imu/data                 | 处理后的 IMU 数据                         | IMU 驱动或滤波节点                                | 姿态估计、融合定位、平衡控制                     |
| /imu/data_raw             | 原始 IMU 数据                             | IMU 驱动                                          | 适合做滤波参数调试和传感器自检                   |
| /imu/mag                  | 磁力计数据                                | IMU 驱动                                          | 航向估计、姿态融合时可能会用到                   |
| /joint_states             | 关节状态，包含各轮子或可动关节的角度/速度 | joint_state_publisher、机器人状态发布器           | RViz 显示模型、TF 链、机械结构联动依赖它         |
| /robot_description        | 机器人 URDF/Xacro 展开后的描述字符串      | robot_state_publisher                             | 机器人模型、TF 树、RViz 机器人显示的核心         |
| /tf                       | 动态坐标变换                              | robot_state_publisher、底盘、IMU、定位节点        | 所有传感器坐标系对齐、导航定位必需               |
| /tf_static                | 静态坐标变换                              | static_transform_publisher、robot_state_publisher | 相机、雷达到 base_link 的固定外参                |
| /JoyState                 | 摇杆或控制状态聚合量，通常用于人机交互    | 遥控/手柄输入节点                                 | 二开时可以映射成模式切换、速度档位等             |
| /joy                      | 手柄原始按键和摇杆数据                    | joy_node                                          | 手柄遥控、模式切换、辅助控制                     |
| /joy/set_feedback         | 手柄震动或反馈控制                        | joy_node / 手柄驱动                               | 适合做报警反馈或状态提示                         |
| /Buzzer                   | 蜂鸣器控制                                | 底盘控制板接口节点                                | 可用于报警、按键反馈、状态提示                   |
| /RGBLight                 | RGB 灯控制                                | 底盘控制板接口节点                                | 可用于运行状态显示、模式指示                     |
| /voltage                  | 电池电压或电源状态                        | 底盘电源监测节点                                  | 低电量告警、续航管理、任务中断保护               |
| /diagnostics              | 系统诊断信息                              | 各类驱动和诊断节点                                | 快速判断相机、雷达、底盘是否正常                 |
| /edition                  | 版本或固件信息类话题                      | 底盘/控制板信息节点                               | 用于识别硬件版本、兼容性判断                     |
| /move_base/cancel         | 取消导航任务                              | Nav2 或兼容导航接口                               | 手动打断导航、任务切换时会用到                   |
| /set_pose                 | 设置初始位姿或位姿校准入口                | 定位/导航相关节点                                 | 用于 AMCL 初始定位、重定位、仿真调试             |
| /rosout                   | ROS 日志输出                              | ROS 2 系统                                        | 调试时重点看错误、告警和运行状态                 |
| /parameter_events         | 参数变更事件                              | ROS 2 系统                                        | 动态参数调试、节点行为调整                       |

## 3. 按功能拆解

### 3.1 底盘控制

核心是 `/cmd_vel`、`/odom`、`/odom_raw`、`/tf` 和 `/tf_static`。导航、手柄控制、键盘控制最后通常都会落到 `/cmd_vel`，底盘再把运动结果反馈成里程计和 TF。

二次开发建议：

- 如果做自定义运动控制，优先兼容 `/cmd_vel`。
- 如果要做轨迹记录或定位融合，优先订阅 `/odom`、`/imu/data` 和 `/tf`。
- 如果要调底盘参数，先看 `/odom_raw` 是否稳定，再看融合后的 `/odom`。

### 3.2 视觉与深度感知

`/camera/color/*`、`/camera/depth/*`、`/camera/ir/*` 和 `/camera/depth/points` 组成了相机数据通道。彩色图像适合识别，深度和点云适合避障、距离估计和三维环境建模。

二次开发建议：

- 视觉检测一般使用 `/camera/color/image_raw`。
- 空间距离判断和障碍物处理一般使用 `/camera/depth/image_raw` 或 `/camera/depth/points`。
- 做相机标定、图像坐标还原时要配套使用对应的 `/camera/*/camera_info`。

### 3.3 姿态与传感器健康

`/imu/data`、`/imu/data_raw`、`/imu/mag` 和 `/diagnostics` 用于判断机器人姿态、角速度、磁场状态和系统健康情况。

二次开发建议：

- 做融合定位时，先确认 `/imu/data` 是否经过滤波。
- 做异常检测时，重点关注 `/diagnostics` 和 `/voltage`。
- 如果航向漂移明显，再检查 `/imu/mag` 是否受到环境干扰。

### 3.4 交互与状态反馈

`/joy`、`/JoyState`、`/joy/set_feedback`、`/Buzzer` 和 `/RGBLight` 属于人机交互和状态提示接口。

二次开发建议：

- 手柄遥控一般直接订阅 `/joy`。
- 如果需要更高层的控制逻辑，可以把 `/JoyState` 当作模式选择或动作状态输入。
- 蜂鸣器和 RGB 灯适合做任务开始、失败、低电量、避障告警等反馈。

### 3.5 导航接口

`/move_base/cancel` 和 `/set_pose` 说明系统已经接入了导航或定位相关功能。

二次开发建议：

- 取消导航任务时使用 `/move_base/cancel`。
- 初始定位、地图重定位或姿态校准时使用 `/set_pose`。

## 4. 开发时最常看的 Topic 组合

1. 运动控制调试：`/cmd_vel`、`/odom`、`/tf`
2. 相机算法开发：`/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/depth/points`
3. 导航调试：`/scan`、`/odom`、`/tf`、`/move_base/cancel`
4. 状态诊断：`/diagnostics`、`/voltage`、`/imu/data`、`/rosout`
5. 模型显示：`/robot_description`、`/joint_states`、`/tf_static`

## 5. 二开建议

- 如果你要新增算法节点，尽量先明确它是吃传感器数据，还是吃状态数据，再决定订阅哪些 topic。
- 如果你要改底盘行为，优先从 `/cmd_vel` 和 `/odom_raw` 入手。
- 如果你要做视觉功能，优先检查相机 topic 是否稳定发布，以及 `camera_info` 是否匹配。
- 如果你要调试模型显示，优先检查 `/robot_description`、`/joint_states` 和 `/tf_static`。

## 6. 备注

本文根据当前一次 `ros2 topic list` 的结果整理，后续如果新节点启动、话题名称变化或新增功能模块，这份文档可以直接继续补充。
