# 2026--service-robot

#### 介绍
本服务机器人采用基于**Jeston nano主板**作为上位机，通过在Ubuntu18.04上搭载*ROS*操作系统，完成赛项规定的任务。包括通过双目摄像头实现二维码Apriltag识别，基于雷达的激光SLAM建图，gmapping多点导航算法等，实现对机器人的运动规划及其控制。

####  硬件层说明

- 上位机：Jetson Nano
- 深度/双目视觉：用于 AprilTag 检测与视觉任务
- 激光雷达：用于建图与定位导航
- 底盘与电机控制：执行线速度/角速度指令

#### 软件应用层架构

- 操作系统：Ubuntu 18.04
- 机器人中间件：ROS（catkin 工作空间）
- 主要功能包：
	- `match_mini`：导航主逻辑、任务流程
	- `robot_slam` / `rplidar_ros`：建图与定位
	- `robot_vision`：AprilTag 等视觉检测
	- `robot_base`：底盘基础控制

#### 赛项流程

1. 启动底盘、传感器、定位导航与视觉检测节点。
2. 发送任务触发信号到 `auto_slam`。
3. `auto_slam_node` 根据按键值选择航点文件与执行策略。
4. 机器人先完成区域 2 弓字形覆盖，再前往检测点等待 AprilTag。
5. 根据识别到的 Tag（1 或 3）决定后续区域清扫顺序。
6. 清扫完成后优先返回第 4 个航点，若不存在则返回原点并停车。



#### 操作说明

1. 准备环境

```bash
cd ~/code_819/robot_ws
source devel/setup.bash
```

2. 按项目脚本依次启动基础能力（示例）

```bash
cd ~/code_819/robot_shell
./3-lidar.launch
./8-navigation.sh
./11-apriltag_detector.sh
```

3. 启动自动导航节点

```bash
rosrun match_mini auto_slam.py
```

4. 触发任务（向 `auto_slam` 发布字符串）

```bash
rostopic pub /auto_slam std_msgs/String "data: 'start'" -1
```

5. 可选：先发布按键主题决定任务模式

```bash
rostopic pub /key_input std_msgs/String "data: 'g'" -1
```

按键说明（来自代码逻辑）：

- `g`：使用 `waypoints5_1.xml`，并先前往初始化点 `(1.9, 1.87)`。
- `h`：使用 `waypoints5_1.xml`，初始 Tag 设为 1。
- `j`：使用 `waypoints5_3.xml`，初始 Tag 设为 3。
- 其他值：默认使用 `waypoints5_1.xml`。

#### 拓展功能

- 基于yolo的人脸识别

**功能概述**

- **主程序文件**: 自动导航与弓字形覆盖清扫逻辑位于 [robot_ws/src/match_mini/scripts/auto_slam.py](robot_ws/src/match_mini/scripts/auto_slam.py)。
- **运行方式**: 以 ROS 节点 `auto_slam_node` 运行，订阅 `auto_slam` 主题触发完整导航任务。
- **核心能力**: 基于 `move_base` action 实现精确点到点导航，结合 XML 航点和局部覆盖路径完成多区域清扫。
- **视觉决策**: 在检测点等待 AprilTag 识别，依据 Tag 结果动态调整后续区域执行顺序。
- **安全结束**: 任务结束自动返回终点（或原点）并发布零速 `Twist()` 停车。

**主要组件与函数**

- **NavigationController**：核心控制类，负责订阅/发布、action 通信、路径规划与任务编排。
- **parse_xml(xml_file)**：解析航点 XML，输出 `[[pos+quat], ...]` 的导航点列表。
- **generate_special_path(start_x, start_y, idex, forward_length=0.75)**：生成 9 点弓字形覆盖轨迹（支持 1/2/3 三种模式）。
- **navigate_to_point(point, index, total)**：下发 `MoveBaseGoal`，等待结果并判断 `GoalStatus`。
- **execute_special_path(start_x, start_y, idex)**：依次执行弓字形轨迹中的每个路径点。
- **navigate_points(points, id)**：Tag 分支下的组合路径流程（含避障过渡点）。
- **start_navigation()**：任务入口，完成“区域2预清扫 -> 检测点识别 -> 分支清扫 -> 返航停车”全流程。
- **callback(msg, controller)**：收到触发后启动任务，若正在执行则忽略重复触发。

**节点通信接口（按代码整理）**

- 订阅话题：
	- `/apriltag_detector/detections`（`robot_vision/AprilTagDetection`）
	- `/odom`（`nav_msgs/Odometry`）
	- `/key_input`（`std_msgs/String`）
	- `/auto_slam`（`std_msgs/String`，任务触发）
- 发布话题：
	- `/initialpose`（`geometry_msgs/PoseWithCovarianceStamped`）
	- `/cmd_vel`（`geometry_msgs/Twist`，用于停车）
- 动作接口：
	- `move_base`（`move_base_msgs/MoveBaseAction`）

**主流程详解（start_navigation）**

1. 根据 `key_input` 选择航点文件（`waypoints5_1.xml` 或 `waypoints5_3.xml`）。
2. 导航到区域 2（`points[1]`）并执行弓字形覆盖（idex=1）。
3. 前往检测点 `(2.41, 2.48)`，朝向 `45°`，开启 AprilTag 等待。
4. 若识别为 `tag==1`：先执行 `points[0]`，再执行 `points[2]`。
5. 若识别为 `tag==3`：先执行 `points[2]`，再执行 `points[0]`。
6. 任务结束优先返回 `points[3]`，否则返回 `(0,0)`；最后发布零速停车。

**运行与集成注意事项**

- **依赖**：`move_base`、`actionlib`、`tf`、`robot_vision/AprilTagDetection` 及常见 ROS 消息包。
- **航点路径**：当前代码中为绝对路径（如 `/home/bcsh/waypoints5_1.xml`），部署时建议改为参数化路径，避免换机后失效。
- **检测等待机制**：检测点处为阻塞等待（循环直到识别到 Tag 或节点退出），建议比赛时确保检测节点稳定发布。
- **导航超时**：单个目标默认等待 60s，超时会记录告警并继续后续逻辑。

**建议优化（可选）**

1. 将 XML 路径、检测点坐标、等待超时改为 ROS 参数。
2. 为 AprilTag 增加等待超时与重试策略，避免无限等待。
3. 增加状态机日志（阶段开始/结束）便于现场定位问题。
4. 在关键动作前后补充 `rostopic echo` 检查脚本。

更多实现细节见 [robot_ws/src/match_mini/scripts/auto_slam.py](robot_ws/src/match_mini/scripts/auto_slam.py)。

#### 目录结构（简要）

- `robot_ws/`：主 ROS 工作空间（导航、底盘、SLAM 等）
- `vision_ws/`：视觉相关工作空间
- `robot_shell/`：常用启动脚本集合
- `waypoints*.xml`：任务航点文件

#### 贡献者
- 
- 
- 