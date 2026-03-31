# 2026--service-robot

#### 介绍
本服务机器人采用基于**Jeston nano主板**作为上位机，通过在Ubuntu18.04上搭载*ROS*操作系统，完成赛项规定的任务。包括通过双目摄像头实现二维码Apriltag识别，基于雷达的激光SLAM建图，gmapping多点导航算法等，实现对机器人的运动规划及其控制。

####  硬件层说明

#### 软件应用层架构

#### 赛项流程

#### 操作说明

#### 拓展功能

**功能概述**

- **主程序文件**: 本仓库的自动导航与弓字形覆盖清扫逻辑位于 [auto_slam.py](auto_slam.py)。
- **运行方式**: 作为一个 ROS 节点 `auto_slam_node` 启动，订阅 `auto_slam` 主题以触发导航流程。
- **核心功能**: 基于 `move_base` action 实现点到点导航，解析 XML 航点文件生成航点序列，并在关键区域执行弓字形（覆盖式）路径以完成局部覆盖清扫。
- **AprilTag 支持**: 在指定检测点等待 AprilTag（二维码）识别，识别结果用于决定后续航点分支逻辑。
- **输入/输出接口**: 订阅 `/apriltag_detector/detections`、`/odom`、`key_input` 和 `auto_slam`；发布 `/initialpose`（初始位姿）和 `/cmd_vel`（速度控制）。

**主要组件与函数**

- **NavigationController**: 程序核心类，封装 ROS 通信、导航动作客户端、航点解析与控制流程。
- **parse_xml(xml_file)**: 解析 XML 格式的航点文件（waypoints），返回航点列表。
- **generate_special_path(start_x, start_y, idex, forward_length=0.75)**: 为指定起点生成弓字形覆盖路径点序列，支持不同模式（idex 1/2/3）。
- **navigate_to_point(point, index, total)**: 构造并发送 `MoveBaseGoal` 到 `move_base`，等待结果并返回成功/失败。
- **execute_special_path(start_x, start_y, idex)**: 依次导航到弓字形路径中的每个点完成覆盖动作。
- **navigate_points(points, id)**: 按照不同场景 id（例如 1 或 3）编排多段导航与局部覆盖流程（包含避障点与分段执行）。
- **start_navigation()**: 导航主流程入口；根据按键选择 XML、前往指定区域、执行弓字形清扫、移动到检测点并等待 AprilTag 识别，随后根据识别结果继续后续航点。
- **callback(msg, controller)**: 订阅 `auto_slam` 主题的触发回调，用于启动导航流程。

**运行与集成注意事项**

- **依赖**: 依赖 ROS（move_base、actionlib）、`robot_vision` 中的 `AprilTagDetection` 消息类型以及标准消息类型（`nav_msgs/Odometry`、`geometry_msgs` 等）。
- **航点文件路径**: 代码中示例使用类似 `/home/bcsh/waypoints5_1.xml` 的绝对路径，运行前请根据实际环境调整为工作空间内或设备上的正确路径（例如仓库根目录下的 `waypoints*.xml`）。
- **配置建议**: 确保 `move_base` 已启动并能接受外部 `MoveBaseGoal`，AprilTag 检测节点在 ` /apriltag_detector/detections` 发布消息以支持自动分支决策。

更多实现细节见 [auto_slam.py](auto_slam.py).

#### 贡献者
- 
- 
- 