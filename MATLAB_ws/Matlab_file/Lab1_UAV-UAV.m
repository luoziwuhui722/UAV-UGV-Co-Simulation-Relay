%% ==================== 主函数 ====================
% 主函数入口：执行无人机与多台移动机器人的协同控制
% 1. 初始化 ROS2 环境，创建发布器/订阅器
% 2. 初始化机器人结构体（定义航点、阈值等）
% 3. 等待所有机器人获取初始位姿
% 4. 进入主循环：
%    - 每 0.1 秒发布一次无人机的轨迹设定点（Offboard 模式）
%    - 每 1 秒执行一次状态检查：获取所有机器人位置、更新无人机状态机、
%      更新小车目标点并发布、打印状态信息
%% 参数设置
publish_rate = 0.1;          % 无人机轨迹点发布周期 (100 ms)
check_interval = 1;          % 状态检查周期 (1 s)
setpoint_counter = 0;

%% ROS2 初始化
[node, offboard_pub, traj_pub, vehicle_pub, localPosSub] = init_ros2();

%% 初始化机器人结构
robots = init_robots(node);

%% 等待所有机器人获取初始位置
[latest_uav_pos, robots] = wait_for_initial_poses(localPosSub, robots, 10);

%% 状态变量
uav_state = 'initial_climb';          % 无人机状态机
wait_start_time = 0;                  % 停留开始时间
round_trip_count = 0;                 % 已完成来回次数
initial_target = [0.0, 0.0, -9.0];
target1 = [-5.0, -10.0, -5.0];
target2 = [5.0, -20.0, -5.0];
current_target = initial_target;      % 当前目标点

%% 数据记录相关
data_log = {};                        % 存储每条记录的 cell 数组
start_clock = tic;                    % 相对时间计时器（秒）
% 注册退出回调，确保中途退出时保存数据
cleanupObj = onCleanup(@() save_data_to_excel(data_log));

fprintf('开始协同控制：无人机 0.1s 发布轨迹点，所有机器人每 %d s 检查状态并发送目标点...\n', check_interval);
last_check_time = tic;

%% 主循环
while true
    timestamp = uint64(posixtime(datetime('now'))*1e6);

    % 发布无人机轨迹点（0.1s 频率）
    publish_uav_trajectory(offboard_pub, traj_pub, vehicle_pub, ...
        uav_state, current_target, initial_target, target1, target2, ...
        setpoint_counter, timestamp);

    % 状态检查中断（check_interval 频率）
    if toc(last_check_time) >= check_interval
        % 获取所有机器人最新位置
        latest_uav_pos = get_uav_position(localPosSub, latest_uav_pos);
        robots = get_robot_poses(robots);

        % 更新无人机状态
        [uav_state, current_target, wait_start_time, round_trip_count] = ...
            update_uav_state(uav_state, latest_uav_pos, current_target, ...
            initial_target, target1, target2, wait_start_time, round_trip_count);

        % 更新小车状态并发送目标点
        robots = update_and_send_robot_goals(robots, node);

        % 打印所有机器人状态
        print_status(uav_state, latest_uav_pos, current_target, robots);

        % ---------- 无中继通信指标计算与记录 ----------
        uav2_pos = [15, 0, 2];   % 固定基站位置
        try
            % 调用函数获取指标数值
            [d, PL_dB, Pr_dBm, SINR_dB, BER, delay_s] = compute_no_relay_metrics(latest_uav_pos, uav2_pos);
            elapsed_time = toc(start_clock);   % 仿真已运行秒数
            % 记录到 cell 数组
            data_row = {elapsed_time, d, PL_dB, Pr_dBm, SINR_dB, BER, delay_s};
            data_log = [data_log; data_row];
            save_data_to_excel(data_log);
        catch ME
            fprintf('通信指标计算失败: %s\n', ME.message);
        end
        % ---------------------------------------------

        % 重置计时器
        last_check_time = tic;
    end

    setpoint_counter = setpoint_counter + 1;
    pause(publish_rate);
end






%% ==================== 子函数 ====================
% ------------------------------------------------------------------------
% 函数名称: init_ros2
% 功能描述: 初始化 ROS2 环境，创建 ROS2 节点，并为无人机创建所需的
%           发布器和订阅器。
% 输入参数: 无
% 输出参数:
%   node          - ROS2 节点句柄
%   offboard_pub  - 无人机 offboard 控制模式发布器
%   traj_pub      - 无人机轨迹设定点发布器
%   vehicle_pub   - 无人机车辆命令发布器
%   localPosSub   - 无人机本地位置订阅器
% ------------------------------------------------------------------------
function [node, offboard_pub, traj_pub, vehicle_pub, localPosSub] = init_ros2()
    setenv('ROS_DOMAIN_ID', '0');
    setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

    node = ros2node("/multi_robot_control");

    offboard_pub = ros2publisher(node, "/fmu/in/offboard_control_mode", "px4_msgs/OffboardControlMode");
    traj_pub     = ros2publisher(node, "/fmu/in/trajectory_setpoint", "px4_msgs/TrajectorySetpoint");
    vehicle_pub  = ros2publisher(node, "/fmu/in/vehicle_command", "px4_msgs/VehicleCommand");

    % 订阅无人机位置
    localPosSub = ros2subscriber(node, "/fmu/out/vehicle_local_position_v1", ...
        "px4_msgs/VehicleLocalPosition", "Reliability", "besteffort");

    % 注意：无人机状态订阅未在后续使用，故不保留
end

% ------------------------------------------------------------------------
% 函数名称: init_robots
% 功能描述: 初始化两辆小车的结构体，为每辆小车定义航点、阈值、ROS2
%           发布器与订阅器，并存储其状态信息。
% 输入参数:
%   node        - ROS2 节点句柄
% 输出参数:
%   robots      - 包含两辆小车信息的结构体数组
% ------------------------------------------------------------------------
function robots = init_robots(node)
    % 定义两辆小车的参数
    robot_defs = {
        struct('name', 'TB3_1', ...
               'goal_topic', '/TB3_1/goal_pose', ...
               'pose_topic', '/TB3_1/amcl_pose', ...
               'waypoints', [-10, 5; -10, -6; -20, -6; -20, 6], ...
               'threshold', 0.2, ...
               'current_idx', 1, ...
               'has_pose', false),
        struct('name', 'TB3_2', ...
               'goal_topic', '/TB3_2/goal_pose', ...
               'pose_topic', '/TB3_2/amcl_pose', ...
               'waypoints', [10, 5; 10, -5; 20, -5; 20, 5], ...
               'threshold', 0.2, ...
               'current_idx', 1, ...
               'has_pose', false)
    };

    % 为每个机器人创建发布器和订阅器
    for i = 1:length(robot_defs)
        robot_defs{i}.goal_pub = ros2publisher(node, robot_defs{i}.goal_topic, 'geometry_msgs/PoseStamped');
        robot_defs{i}.pose_sub = ros2subscriber(node, robot_defs{i}.pose_topic, 'geometry_msgs/PoseWithCovarianceStamped', ...
                                                'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);
        robot_defs{i}.last_pose = [];  % 占位
    end
    robots = robot_defs;
end

% ------------------------------------------------------------------------
% 函数名称: wait_for_initial_poses
% 功能描述: 阻塞等待所有机器人（无人机+小车）获取初始位姿，直到全部
%           收到有效位置消息或超时。
% 输入参数:
%   localPosSub - 无人机位置订阅器
%   robots      - 小车结构体数组
%   timeout     - 超时时间（秒）
% 输出参数:
%   latest_uav_pos - 无人机初始位置 [x, y, z]
%   robots         - 更新了初始位姿的小车结构体数组
% ------------------------------------------------------------------------
function [latest_uav_pos, robots] = wait_for_initial_poses(localPosSub, robots, timeout)
    fprintf('等待获取无人机和小车初始位置...\n');
    start_time = tic;
    latest_uav_pos = [NaN, NaN, NaN];

    while true
        all_received = true;

        % 检查无人机位置
        try
            pos_msg = receive(localPosSub, 0.01);
            if ~isempty(pos_msg)
                latest_uav_pos = double([pos_msg.x, pos_msg.y, pos_msg.z]);
                fprintf('成功获取无人机位置： (%.2f, %.2f, %.2f)\n', latest_uav_pos(1), latest_uav_pos(2), latest_uav_pos(3));
            else
                all_received = false;
            end
        catch
            all_received = false;
        end

        % 检查小车位置
        for i = 1:length(robots)
            pose_msg = robots{i}.pose_sub.LatestMessage;
            if ~isempty(pose_msg)
                robots{i}.has_pose = true;
                robots{i}.last_pose = pose_msg;
                fprintf('成功获取 %s 位置： (%.2f, %.2f)\n', robots{i}.name, ...
                        pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
            else
                all_received = false;
            end
        end

        if all_received
            fprintf('所有机器人位置已获取！\n');
            break;
        end
        if toc(start_time) > timeout
            error('超时：部分机器人未获取到位置。请检查 AMCL 和 PX4 是否运行正常。');
        end
        pause(0.2);
    end
end

% ------------------------------------------------------------------------
% 函数名称: publish_uav_trajectory
% 功能描述: 发布无人机 offboard 控制模式和轨迹设定点，并在第 10 次
%           循环时发送 offboard 模式切换和解锁命令（仅一次）。
% 输入参数:
%   offboard_pub    - offboard 控制模式发布器
%   traj_pub        - 轨迹设定点发布器
%   vehicle_pub     - 车辆命令发布器
%   uav_state       - 无人机当前状态
%   current_target  - 当前目标点（用于 waiting/finished 状态）
%   initial_target  - 初始爬升点
%   target1         - 目标点1
%   target2         - 目标点2
%   setpoint_counter- 已发布轨迹点数计数器
%   timestamp       - 当前时间戳（微秒）
% 输出参数: 无
% ------------------------------------------------------------------------
function publish_uav_trajectory(offboard_pub, traj_pub, vehicle_pub, ...
        uav_state, current_target, initial_target, target1, target2, ...
        setpoint_counter, timestamp)

    % 1. OffboardControlMode 消息
    offboard_msg = ros2message(offboard_pub);
    offboard_msg.position     = true;
    offboard_msg.velocity     = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude     = false;
    offboard_msg.body_rate    = false;
    offboard_msg.timestamp    = timestamp;
    send(offboard_pub, offboard_msg);

    % 2. TrajectorySetpoint 消息（根据状态选择目标点）
    traj_msg = ros2message(traj_pub);
    switch uav_state
        case 'initial_climb'
            target = initial_target;
        case 'going_to_1'
            target = target1;
        case 'going_to_2'
            target = target2;
        case {'waiting', 'finished'}
            target = current_target;  % 保持不变
        otherwise
            target = current_target;
    end
    traj_msg.position     = single(target);
    traj_msg.velocity     = single([0,0,0]);
    traj_msg.acceleration = single([0,0,0]);
    traj_msg.jerk         = single([0,0,0]);
    traj_msg.yaw          = single(-3.14);
    traj_msg.yawspeed     = single(0.0);
    traj_msg.timestamp    = timestamp;
    send(traj_pub, traj_msg);

    % 3. 设置 Offboard 模式并解锁（仅一次）
    if setpoint_counter == 10
        % Offboard mode
        mode_msg = ros2message(vehicle_pub);
        mode_msg.command          = uint32(176);
        mode_msg.param1           = single(1);
        mode_msg.param2           = single(6);
        mode_msg.target_system    = uint8(1);
        mode_msg.target_component = uint8(1);
        mode_msg.source_system    = uint8(1);
        mode_msg.source_component = uint16(1);
        mode_msg.from_external    = true;
        mode_msg.timestamp        = timestamp;
        send(vehicle_pub, mode_msg);

        % Arm
        arm_msg = ros2message(vehicle_pub);
        arm_msg.command          = uint32(400);
        arm_msg.param1           = single(1.0);
        arm_msg.param2           = single(0.0);
        arm_msg.target_system    = uint8(1);
        arm_msg.target_component = uint8(1);
        arm_msg.source_system    = uint8(1);
        arm_msg.source_component = uint16(1);
        arm_msg.from_external    = true;
        arm_msg.timestamp        = timestamp;
        send(vehicle_pub, arm_msg);

        disp("Offboard mode set and vehicle armed.");
    end
end

% ------------------------------------------------------------------------
% 函数名称: get_uav_position
% 功能描述: 尝试从订阅器获取无人机的最新位置，若接收成功则更新，
%           否则保留上次位置。
% 输入参数:
%   localPosSub   - 无人机位置订阅器
%   latest_uav_pos- 上一次保存的无人机位置（NaN 表示未知）
% 输出参数:
%   latest_uav_pos- 更新后的无人机位置（若无新消息则不变）
% ------------------------------------------------------------------------
function latest_uav_pos = get_uav_position(localPosSub, latest_uav_pos)
    try
        pos_msg = receive(localPosSub, 0.01);
        if ~isempty(pos_msg)
            latest_uav_pos = double([pos_msg.x, pos_msg.y, pos_msg.z]);
        end
    catch
        % 无新消息，保留原有值
    end
end

% ------------------------------------------------------------------------
% 函数名称: get_robot_poses
% 功能描述: 从每辆小车的订阅器中获取最新位姿，并更新结构体中的
%           last_pose 和 has_pose 标志。
% 输入参数:
%   robots      - 小车结构体数组（包含 pose_sub）
% 输出参数:
%   robots      - 更新了位姿信息的结构体数组
% ------------------------------------------------------------------------
function robots = get_robot_poses(robots)
    for i = 1:length(robots)
        pose_msg = robots{i}.pose_sub.LatestMessage;
        if ~isempty(pose_msg)
            robots{i}.last_pose = pose_msg;
            robots{i}.has_pose = true;
        end
    end
end

% ------------------------------------------------------------------------
% 函数名称: update_uav_state
% 功能描述: 根据无人机当前位置和当前状态，更新状态机并执行相应的
%           动作（到达检测、停留计时、往返计数）。
% 输入参数:
%   uav_state       - 当前状态
%   latest_uav_pos  - 无人机当前位置
%   current_target  - 当前目标点（用于等待/完成状态）
%   initial_target  - 初始爬升点
%   target1         - 目标点1
%   target2         - 目标点2
%   wait_start_time - 停留开始时间（Unix 秒）
%   round_trip_count- 已完成来回次数
% 输出参数:
%   uav_state       - 更新后的状态
%   current_target  - 更新后的当前目标点
%   wait_start_time - 更新后的停留开始时间
%   round_trip_count- 更新后的来回次数
% ------------------------------------------------------------------------
function [uav_state, current_target, wait_start_time, round_trip_count] = ...
        update_uav_state(uav_state, latest_uav_pos, current_target, ...
        initial_target, target1, target2, wait_start_time, round_trip_count)

    if strcmp(uav_state, 'finished') || any(isnan(latest_uav_pos))
        return;
    end

    switch uav_state
        case 'initial_climb'
            if abs(latest_uav_pos(1) - initial_target(1)) < 1.0 && ...
               abs(latest_uav_pos(2) - initial_target(2)) < 1.0 && ...
               abs(latest_uav_pos(3) - initial_target(3)) < 0.5
                uav_state = 'going_to_1';
                current_target = target1;
                disp('Reached initial climb point (0,0,-9). Now flying to target point 1.');
            end

        case 'going_to_1'
            if abs(latest_uav_pos(1) - target1(1)) < 0.1 && ...
               abs(latest_uav_pos(2) - target1(2)) < 0.1 && ...
               abs(latest_uav_pos(3) - target1(3)) < 0.1
                uav_state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target1;
                disp('Reached target point 1. Waiting for 3 seconds...');
            end

        case 'going_to_2'
            if abs(latest_uav_pos(1) - target2(1)) < 0.1 && ...
               abs(latest_uav_pos(2) - target2(2)) < 0.1 && ...
               abs(latest_uav_pos(3) - target2(3)) < 0.1
                uav_state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target2;
                disp('Reached target point 2. Waiting for 3 seconds...');
            end

        case 'waiting'
            if posixtime(datetime('now')) - wait_start_time >= 10.0
                if isequal(current_target, target1)
                    uav_state = 'going_to_2';
                    disp('Wait finished. Flying to target point 2.');
                elseif isequal(current_target, target2)
                    round_trip_count = round_trip_count + 1;
                    fprintf('Round trip %d completed.\n', round_trip_count);
                    if round_trip_count >= 10
                        uav_state = 'finished';
                        disp('Completed 10 round trips. Stopping at target point 1.');
                    else
                        uav_state = 'going_to_1';
                        disp('Wait finished. Returning to target point 1.');
                    end
                end
            end
    end
end

% ------------------------------------------------------------------------
% 函数名称: update_and_send_robot_goals
% 功能描述: 对每辆小车进行到达检测，若到达当前航点则切换到下一个；
%           然后向小车发布目标点（无论是否到达均发送，确保小车持续
%           获得指令）。
% 输入参数:
%   robots      - 小车结构体数组（包含当前状态、航点、发布器等）
%   node        - ROS2 节点句柄（用于获取时间戳）
% 输出参数:
%   robots      - 更新了 current_idx 和可能的目标点后的结构体数组
% ------------------------------------------------------------------------
function robots = update_and_send_robot_goals(robots, node)
    for i = 1:length(robots)
        robot = robots{i};
        if ~robot.has_pose
            continue;
        end

        current_x = robot.last_pose.pose.pose.position.x;
        current_y = robot.last_pose.pose.pose.position.y;
        waypoint = robot.waypoints(robot.current_idx, :);
        dist = sqrt((current_x - waypoint(1))^2 + (current_y - waypoint(2))^2);

        % 到达判断
        if dist < robot.threshold
            fprintf('%s 已到达目标点 %d: (%.2f, %.2f)\n', ...
                    robot.name, robot.current_idx, waypoint(1), waypoint(2));
            robot.current_idx = robot.current_idx + 1;
            if robot.current_idx > size(robot.waypoints, 1)
                robot.current_idx = 1;  % 循环
            end
            waypoint = robot.waypoints(robot.current_idx, :);
            fprintf('%s 开始前往下一个目标点 %d: (%.2f, %.2f)\n', ...
                    robot.name, robot.current_idx, waypoint(1), waypoint(2));
        end

        % 发送目标点（无论是否到达，都发送当前目标点）
        goal_msg = ros2message(robot.goal_pub);
        goal_msg.header.stamp = ros2time(node, "now");
        goal_msg.header.frame_id = 'map';
        goal_msg.pose.position.x = waypoint(1);
        goal_msg.pose.position.y = waypoint(2);
        goal_msg.pose.position.z = 0.0;
        goal_msg.pose.orientation.w = 1.0;
        send(robot.goal_pub, goal_msg);

        robots{i} = robot;
    end
end

% ------------------------------------------------------------------------
% 函数名称: print_status
% 功能描述: 打印当前所有机器人（无人机+小车）的位置、状态和目标点
%           信息，用于调试和监控。
% 输入参数:
%   uav_state       - 无人机状态
%   latest_uav_pos  - 无人机当前位置
%   current_target  - 无人机当前目标点
%   robots          - 小车结构体数组
% 输出参数: 无
% ------------------------------------------------------------------------
function print_status(uav_state, latest_uav_pos, current_target, robots)
    fprintf('\n===== 当前状态 (检查时刻) =====\n');

    % 无人机
    if ~any(isnan(latest_uav_pos))
        fprintf('无人机 | 状态: %s | 位置: [%.2f, %.2f, %.2f] | 目标: [%.2f, %.2f, %.2f]\n', ...
                uav_state, latest_uav_pos(1), latest_uav_pos(2), latest_uav_pos(3), ...
                current_target(1), current_target(2), current_target(3));
    else
        fprintf('无人机 | 状态: %s | 等待位置数据... | 目标: [%.2f, %.2f, %.2f]\n', ...
                uav_state, current_target(1), current_target(2), current_target(3));
    end

    % 小车
    for i = 1:length(robots)
        robot = robots{i};
        if robot.has_pose
            current_x = robot.last_pose.pose.pose.position.x;
            current_y = robot.last_pose.pose.pose.position.y;
            waypoint = robot.waypoints(robot.current_idx, :);
            dist = sqrt((current_x - waypoint(1))^2 + (current_y - waypoint(2))^2);
            fprintf('%s | 位置: [%.2f, %.2f] | 当前目标点 %d: [%.2f, %.2f] | 距离: %.2f\n', ...
                    robot.name, current_x, current_y, robot.current_idx, waypoint(1), waypoint(2), dist);
        else
            fprintf('%s | 等待位置数据...\n', robot.name);
        end
    end
    fprintf('================================\n\n');
end

% ==================== 无中继通信指标计算 ====================

% ------------------------------------------------------------------------
% 函数名称: compute_no_relay_metrics
% 功能描述: 无中继场景（UAV1 → UAV2），UAV1 在原材料区，UAV2 固定位置。
%           输入为无人机原始坐标（来自 /fmu/out/vehicle_local_position_v1），
%           函数内部转换为世界坐标后再计算通信指标。
% 输入参数:
%   uav1_raw    - UAV1 原始坐标 [x, y, z]（无人机坐标系）
%   uav2_world  - UAV2 世界坐标 [15, 0, 10]（固定）
% 输出参数:
%   d           - 距离 (米)
%   PL_dB       - 路径损耗 (dB)
%   Pr_dBm      - 接收功率 (dBm)
%   SINR_dB     - 信干噪比 (dB)
%   BER         - 误码率
%   delay_s     - 端到端延迟 (秒)
% ------------------------------------------------------------------------
function [d, PL_dB, Pr_dBm, SINR_dB, BER, delay_s] = compute_no_relay_metrics(uav1_raw, uav2_world)
    % 固定系统参数
    Pt_dBm = 20;            % 发射功率 dBm
    PL0_dB = 40;            % 1米处路径损耗 dB
    c = 3e8;                % 光速 m/s
    hop_delay = 0.001;      % 每跳处理延迟 1 ms（直连为一跳）

    % 检查位置有效性
    if any(isnan(uav1_raw)) || any(isinf(uav1_raw))
        error('UAV1 位置无效，无法计算');
    end

    % 将无人机原始坐标转换为世界坐标
    % 原始坐标: uav1_raw = [x_uav, y_uav, z_uav]
    % 世界坐标: world_x = y_uav, world_y = x_uav, world_z = -z_uav
    uav1_world = [uav1_raw(2), uav1_raw(1), -uav1_raw(3)];

    % ------------------- 原材料区参数（随机生成） -------------------
    n = 2.5 + 0.5 * rand();               % 路径损耗指数 2.5~3.0
    noise_rise = 6 + 3 * rand();          % 背景噪声抬升 6~9 dB
    N0_dBm = -100 + noise_rise;           % 背景噪声 -94~-91 dBm
    m = 1.125;                            % Nakagami-m 固定中值（原未定义，现补充）
    
    is_blocked = true;                    % 原材料区有遮挡
    if is_blocked
        n_eff = n + 0.5;
    else
        n_eff = n;
    end

    % 计算链路指标
    [SINR_lin, SINR_dB, Pr_dBm, PL_dB, fade, d] = ...
        calc_single_link(uav1_world, uav2_world, Pt_dBm, PL0_dB, n_eff, N0_dBm, m);

    % 误码率和延迟
    BER = 0.5 * exp(-SINR_lin);           % QPSK 近似
    delay_s = d / c + hop_delay;          % 端到端延迟 (s)

    % 打印结果（保留原有输出）
    fprintf('【无中继通信指标】(UAV1 → UAV2)\n');
    fprintf('=== 直连链路 (原材料区) ===\n');
    fprintf('  距离: %.2f m\n', d);
    fprintf('  路径损耗指数: %.2f (遮挡修正后 %.2f) | 背景噪声: %.1f dBm\n', n, n_eff, N0_dBm);
    fprintf('  Nakagami-m: %.3f | 小尺度衰落因子: %.3f\n', m, fade);
    fprintf('  路径损耗: %.2f dB | 接收功率: %.2f dBm | SINR: %.2f dB\n', PL_dB, Pr_dBm, SINR_dB);
    fprintf('  BER: %.2e | 延迟: %.3f ms\n', BER, delay_s * 1000);
    fprintf('  -------------------------------------------------\n');
end

% ------------------------------------------------------------------------
% 辅助函数: calc_single_link
% 功能描述: 使用给定的信道参数计算单条链路指标
% 输入参数:
%   tx_pos   - 发射端世界坐标 [x,y,z]
%   rx_pos   - 接收端世界坐标 [x,y,z]
%   Pt_dBm   - 发射功率 (dBm)
%   PL0_dB   - 参考距离1米处的路径损耗 (dB)
%   n_eff    - 有效路径损耗指数（已考虑遮挡）
%   N0_dBm   - 背景噪声 (dBm)
%   m        - Nakagami-m 参数（此处保留参数但未使用，改用确定性衰落）
% 输出参数:
%   SINR_linear - SINR (线性值)
%   SINR_dB     - SINR (dB)
%   Pr_dBm      - 接收功率 (dBm)
%   PL_dB       - 路径损耗 (dB)
%   fade        - 小尺度衰落因子 (线性)
%   d           - 距离 (米)
% ------------------------------------------------------------------------
function [SINR_linear, SINR_dB, Pr_dBm, PL_dB, fade, d] = ...
        calc_single_link(tx_pos, rx_pos, Pt_dBm, PL0_dB, n_eff, N0_dBm, m)
    
    % 距离
    d = norm(tx_pos - rx_pos);
    if d < 1, d = 1; end
    
    % 路径损耗
    PL_dB = PL0_dB + 10 * n_eff * log10(d);
    
    % 小尺度衰落因子：基于距离的确定性函数（替代随机 Nakagami）
    fc = 2.4e9;          % 载波频率 2.4 GHz（可根据需要调整）
    lambda = 3e8 / fc;   % 波长 ≈ 0.125 m
    A = 0.1;             % 衰落深度，控制 SINR 波动幅度 (0 < A < 1)
    fade = sqrt(1 + A * sin(2*pi*d / lambda));
    
    % 接收功率
    Pt_linear = 10^((Pt_dBm - 30)/10);          % W
    PL_linear = 10^(-PL_dB / 10);
    Pr_linear = Pt_linear * PL_linear * fade^2;
    Pr_dBm = 10 * log10(Pr_linear) + 30;
    
    % 噪声功率 (线性)
    N0_linear = 10^((N0_dBm - 30)/10);
    
    % SINR (忽略干扰)
    SINR_linear = Pr_linear / N0_linear;
    SINR_dB = 10 * log10(max(SINR_linear, 1e-10));
end

% ------------------------------------------------------------------------
% 函数名称: save_data_to_excel
% 功能描述: 将记录的数据写入 Excel 文件（在程序退出时自动调用）
% 输入参数:
%   data_log - cell 数组，每行包含 [时间, 距离, PL, Pr, SINR, BER, 延迟]
% ------------------------------------------------------------------------
function save_data_to_excel(data_log)
    if isempty(data_log)
        fprintf('没有数据需要保存。\n');
        return;
    end
    
    % 创建表格
    colNames = {'Time_s', 'Distance_m', 'PathLoss_dB', 'RxPower_dBm', ...
                          'SINR_dB', 'BER', 'Delay_s'};

    
    T = cell2table(data_log, 'VariableNames', colNames);
    filename = 'Lab1.xlsx';
    try
        writetable(T, filename);
        fprintf('数据已保存到 %s (共 %d 条记录)\n', filename, height(T));
    catch ME
        fprintf('保存 Excel 文件失败: %s\n', ME.message);
    end
end