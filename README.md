# UAV-UGV-Co-Simulation-Relay
📖 项目简介
本项目构建了一个空地协同联合仿真平台，用于验证工业物流场景下基于地面无人车（UGV）作为移动中继节点的动态中继通信策略。平台集成了：
● 🤖 Gazebo —— 高保真机器人物理仿真环境
● ✈️ PX4 SITL —— 无人机软件在环仿真
● 🚗 Nav2 —— 地面无人车自主导航栈
● 📡 MATLAB —— 控制中心、信道质量计算与中继切换控制
通过联合仿真，可实时计算无人机-基站直连链路及经UGV中继链路的信干噪比（SINR），并执行动态中继切换，最终定量评估通信可靠性提升效果。
🎥 效果演示
空地协同联合仿真运行效果见视频：效果演示.webm
仿真场景包含原材料区与加工区两个功能区域，无人机在任务点间往返飞行，两辆无人车沿预设路径巡逻，控制中心实时切换最优通信链路。
📁 目录结构
UAV-UGV-Co-Simulation-Relay
├── MATLAB_ws                       # MATLAB 工作空间（控制中心）
│   ├── matlab_msg_gen              # ROS 2 自定义消息生成
│   ├── px4_msgs                    # PX4 消息定义
│   └── Matlab_file                 # 核心控制脚本
│       ├── Factory.m
│       ├── Lab1_UAV-UAV.m          # 场景A（无中继）
│       ├── Lab2_UGV1.m             # 场景B1（UGV1作为固定中继-原材料区）
│       ├── Lab2_UGV2.m             # 场景B2（UGV2作为固定中继-加工区）
│       ├── Lab3.m                  # 场景C（动态中继）
│       ├── test.xlsx               # 场景C（动态中继得到的实验数据）
│       ├── uav_ID.m
│       ├── uav_initpoint_2point_closeloop.m
│       ├── uav_px4_recieve.m
│       ├── uav_px4_takeoff.m
│       ├── uav_takeoff.m
│       ├── uav_timer_2point.m
│       ├── uav_to_2point_closeloop.m
│       ├── uav_to_2point_openloop.m
│       ├── ugv_2car.m
│       ├── ugv_TB3_1_square_continue.m
│       ├── ugv_TB3_1_square.m
│       └── ugv_TB3_1_to_point.m
└── ROS2_ws                                             # ROS2工作空间
    ├── build
    ├── install
    ├── log
    └── src                 
        ├── px4_msgs
        ├── px4_ros_com
        └── navigation                                  # 仿真启动与导航功能包
            ├── config                                  # 无人车导航参数配置
            │   └── nav2_params.yaml                    
            ├── launch                                  # 启动文件
            │   ├── 1uav_2ugv.launch.py                 # 总启动文件（一键启动全仿真）
            │   ├── gazebo_multi_nav2_world.launch.py
            │   ├── iris_car.launch.py
            │   ├── nav2_bringup                        # 无人车 Nav2 导航启动
            │   │   ├── bringup_launch.py
            │   │   ├── localization_launch.py
            │   │   ├── navigation_launch.py
            │   │   ├── rviz_launch.py
            │   │   └── slam_launch.py
            │   └── px4.launch.py                       # 无人机 PX4 SITL 启动
            ├── maps                                    # 无人车导航静态地图
            │   ├── Factory.pgm                         
            │   ├── Factory.yaml
            │   ├── room.pgm
            │   └── room.yaml
            ├── models                                  # 三维模型
            │   ├── Factory                             # 工业物流工厂模型
            │   │   ├── Factory.dae
            │   │   ├── model.config
            │   │   └── model.sdf
            │   ├── ground
            │   │   ├── ground.dae
            │   │   ├── ground.jpg
            │   │   ├── model.config
            │   │   └── model.sdf
            │   ├── turtlebot3_burger                   # 无人车模型
            │   │   ├── meshes
            │   │   │   ├── burger_base.dae
            │   │   │   ├── lds.dae
            │   │   │   └── tire.dae
            │   │   ├── model-1_4.sdf
            │   │   ├── model.config
            │   │   └── model.sdf
            │   └── turtlebot3_waffle
            │       ├── meshes
            │       │   ├── lds.dae
            │       │   ├── r200.dae
            │       │   ├── tire.dae
            │       │   └── waffle_base.dae
            │       ├── model-1_4.sdf
            │       ├── model.config
            │       ├── model_nocamera.sdf
            │       └── model.sdf
            ├── navigation
            │   └── __init__.py
            ├── package.xml
            ├── resource
            │   └── navigation
            ├── rviz                                    # RViz 可视化配置
            │   ├── multi_nav2_default_view.rviz
            │   └── tb3_navigation2.rviz
            ├── test
            │   ├── test_copyright.py
            │   ├── test_flake8.py
            │   └── test_pep257.py
            ├── urdf
            │   ├── common_properties.urdf
            │   ├── turtlebot3_burger.urdf
            │   └── turtlebot3_waffle.urdf
            └── worlds                                  # Gazebo 世界文件
                ├── CarMap.world
                └── Factory.world                        
⚙️ 环境依赖
组件	版本/说明
操作系统	Ubuntu 22.04 
ROS 2	Humble Hawksbill
Gazebo	Gazebo Classic 11
PX4-Autopilot	v1.17.0
MATLAB	R2024a ，需安装 ROS Toolbox
Nav2	ROS 2 Humble 发行版自带
TurtleBot3	ROS 2 Humble 仿真包
🚀 快速开始
1. 克隆代码仓库
git clone https://github.com/luoziwuhui722/UAV-UGV-Co-Simulation-Relay.git
cd UAV-UGV-Co-Simulation-Relay
2. 编译 ROS 2 工作空间
cd ROS2_ws
colcon build 
source install/setup.bash
3. 启动联合仿真环境
终端 1：启动 Micro-XRCE-DDS Agent（桥接 PX4 与 ROS 2）
MicroXRCEAgent udp4 -p 8888
终端 2：启动 Gazebo + PX4 SITL + Nav2（一键启动）
cd ROS2_ws
source install/setup.bash
ros2 launch navigation 1uav_2ugv.launch.py
此时 Gazebo 界面将显示工厂仿真环境，Rviz 显示无人车导航状态。
4. 运行控制中心
在 MATLAB 中打开 MATLAB_ws/Matlab_file/ 目录，根据需要选择运行脚本：
脚本文件	对应实验场景	说明
Lab1_UAV-UAV.m	场景A（无中继）	无人机与基站直连，基准对照
Lab2_UGV1.m	场景B1（固定中继-原材料区）	始终经 UGV1 中继
Lab2_UGV2.m	场景B2（固定中继-加工区）	始终经 UGV2 中继
Lab3.m	场景C（动态中继）	实时切换最优链路，核心策略
5. 观察实验与数据记录
● 仿真运行时，控制中心 MATLAB 命令窗口将实时打印当前链路选择、SINR 值及飞行阶段。
● 实验数据自动保存至 test.xlsx（Lab3 场景），包含时间戳、SINR、误码率、延迟等指标。
● 可在 RViz 或 Gazebo 中观察机器人运动轨迹与中继切换行为。
