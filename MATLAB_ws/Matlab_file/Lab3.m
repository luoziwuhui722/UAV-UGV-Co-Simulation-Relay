%% ==================== 主函数：动态中继场景 ====================
% 实验目的：UAV1 在原材料区飞行，基站 UAV2 固定于加工区。
% 中继节点：UGV1（原材料区巡逻）、UGV2（加工区巡逻）。
% 动态中继策略：每个时间步计算三条链路 SINR，若当前链路 SINR 平均值低于 25 dB，
% 则切换到当前 SINR 最高的链路。
% 记录每个时间步的选中链路、SINR、BER、延迟，并保存到 Excel。

%% 参数设置
publish_rate = 0.1;          % 无人机轨迹点发布周期 (100 ms)
check_interval = 1;          % 状态检查周期 (1 s)
setpoint_counter = 0;

%% ROS2 初始化
[node, offboard_pub, traj_pub, vehicle_pub, localPosSub] = init_ros2();

%% 初始化机器人结构（两辆小车：UGV1 原材料区，UGV2 加工区）
robots = init_robots(node);

%% 等待所有机器人获取初始位置
[latest_uav_pos, robots] = wait_for_initial_poses(localPosSub, robots, 10);

%% 无人机状态变量（与之前相同，但悬停时间改为3秒，往返10次）
uav_state = 'initial_climb';
wait_start_time = 0;
round_trip_count = 0;
initial_target = [0.0, 0.0, -9.0];
target1 = [-5.0, -10.0, -5.0];   % RM1
target2 = [5.0, -20.0, -5.0];    % RM2
current_target = initial_target;

%% 动态中继状态变量
current_link = 'direct';          % 可选: 'direct', 'ugv1', 'ugv2'
last_sinr_current = NaN;          % 当前链路的上一次 SINR (dB)
THRESHOLD_SINR_AVG = 25;          % 切换阈值（平均 SINR 低于此值触发切换）

%% 数据记录
data_log = {};
start_clock = tic;

fprintf('开始动态中继实验：无人机 0.1s 发布轨迹点，状态检查每 %d s\n', check_interval);
last_check_time = tic;

%% 主循环
while true
    timestamp = uint64(posixtime(datetime('now'))*1e6);
    
    % 发布无人机轨迹点
    publish_uav_trajectory(offboard_pub, traj_pub, vehicle_pub, ...
        uav_state, current_target, initial_target, target1, target2, ...
        setpoint_counter, timestamp);
    
    % 状态检查（1秒一次）
    if toc(last_check_time) >= check_interval
        % 1. 获取最新位置
        latest_uav_pos = get_uav_position(localPosSub, latest_uav_pos);
        robots = get_robot_poses(robots);
        
        % 2. 更新无人机状态机（含悬停等待3秒，10次往返）
        [uav_state, current_target, wait_start_time, round_trip_count] = ...
            update_uav_state(uav_state, latest_uav_pos, current_target, ...
            initial_target, target1, target2, wait_start_time, round_trip_count);
        
        % 3. 更新小车目标点（保持巡逻）
        robots = update_and_send_robot_goals(robots, node);
        
        % 4. 打印当前状态（可选）
        print_status(uav_state, latest_uav_pos, current_target, robots);
        
        % ---------- 动态中继通信指标计算 ----------
        % 坐标转换
        uav1_world = [latest_uav_pos(2), latest_uav_pos(1), -latest_uav_pos(3)];
        uav2_pos = [15, 0, 2];   % 基站固定位置
        
        if robots{1}.has_pose && robots{2}.has_pose
            ugv1_pos = [robots{1}.last_pose.pose.pose.position.x, ...
                        robots{1}.last_pose.pose.pose.position.y, 0];
            ugv2_pos = [robots{2}.last_pose.pose.pose.position.x, ...
                        robots{2}.last_pose.pose.pose.position.y, 0];
            
            % 计算三条链路指标
            [sinr_direct, ber_direct, delay_direct] = compute_direct_link(uav1_world, uav2_pos);
            [sinr_ugv1, ber_ugv1, delay_ugv1] = compute_relay_link(uav1_world, ugv1_pos, uav2_pos, 'raw');
            [sinr_ugv2, ber_ugv2, delay_ugv2] = compute_relay_link(uav1_world, ugv2_pos, uav2_pos, 'proc');
            
            % 收集当前所有链路 SINR
            links = {'direct', 'ugv1', 'ugv2'};
            sinr_vals = [sinr_direct, sinr_ugv1, sinr_ugv2];
            
            % 获取当前链路的本次 SINR
            switch current_link
                case 'direct'
                    current_sinr = sinr_direct;
                case 'ugv1'
                    current_sinr = sinr_ugv1;
                case 'ugv2'
                    current_sinr = sinr_ugv2;
            end
            
            % 动态切换决策
            if ~isnan(last_sinr_current)
                avg_sinr = (last_sinr_current + current_sinr) / 2;
                if avg_sinr < THRESHOLD_SINR_AVG
                    % 切换到当前 SINR 最高的链路
                    [max_sinr, idx] = max(sinr_vals);
                    new_link = links{idx};
                    if ~strcmp(new_link, current_link)
                        fprintf('[切换] %.2f s: 链路 %s 平均SINR=%.2f dB < %.0f dB，切换到 %s (SINR=%.2f dB)\n', ...
                                toc(start_clock), current_link, avg_sinr, THRESHOLD_SINR_AVG, new_link, max_sinr);
                        current_link = new_link;
                        last_sinr_current = NaN;   % 刚切换，无历史值
                    else
                        % 虽然平均值低于阈值但当前链路仍是最优，不切换
                        last_sinr_current = current_sinr;
                    end
                else
                    last_sinr_current = current_sinr;
                end
            else
                % 首次或刚切换后，记录本次 SINR
                last_sinr_current = current_sinr;
            end
            
            % 确定最终使用的链路指标
            switch current_link
                case 'direct'
                    final_sinr = sinr_direct;
                    final_ber = ber_direct;
                    final_delay = delay_direct;
                case 'ugv1'
                    final_sinr = sinr_ugv1;
                    final_ber = ber_ugv1;
                    final_delay = delay_ugv1;
                case 'ugv2'
                    final_sinr = sinr_ugv2;
                    final_ber = ber_ugv2;
                    final_delay = delay_ugv2;
            end
            
            elapsed_time = toc(start_clock);
            data_row = {elapsed_time, current_link, final_sinr, final_ber, final_delay, ...
                        sinr_direct, sinr_ugv1, sinr_ugv2};
            data_log = [data_log; data_row];
            
            % 保存到 Excel
            save_dynamic_relay_data(data_log);
            
            % 打印本次决策（可选）
            fprintf('[动态中继] 选中: %s | SINR=%.2f dB | BER=%.2e | Delay=%.3f ms\n', ...
                    current_link, final_sinr, final_ber, final_delay*1000);
        else
            fprintf('等待小车位置...\n');
        end
        % --------------------------------------------
        
        last_check_time = tic;
    end
    
    setpoint_counter = setpoint_counter + 1;
    pause(publish_rate);
end

%% ==================== 子函数 ====================
% 以下函数大部分与 Lab2_UGV1.m 相同，仅修改了悬停等待时间（3秒）和往返次数（10）
% 以及新增动态中继相关函数

% ------------------------------------------------------------------------
function [node, offboard_pub, traj_pub, vehicle_pub, localPosSub] = init_ros2()
    setenv('ROS_DOMAIN_ID', '0');
    setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
    node = ros2node("/multi_robot_control");
    offboard_pub = ros2publisher(node, "/fmu/in/offboard_control_mode", "px4_msgs/OffboardControlMode");
    traj_pub     = ros2publisher(node, "/fmu/in/trajectory_setpoint", "px4_msgs/TrajectorySetpoint");
    vehicle_pub  = ros2publisher(node, "/fmu/in/vehicle_command", "px4_msgs/VehicleCommand");
    localPosSub = ros2subscriber(node, "/fmu/out/vehicle_local_position_v1", ...
        "px4_msgs/VehicleLocalPosition", "Reliability", "besteffort");
end

% ------------------------------------------------------------------------
function robots = init_robots(node)
    robot_defs = {
        struct('name', 'UGV_RM', ...      % 原材料区小车
               'goal_topic', '/TB3_1/goal_pose', ...
               'pose_topic', '/TB3_1/amcl_pose', ...
               'waypoints', [-10, 5; -10, -6; -20, -6; -20, 6], ...
               'threshold', 0.2, 'current_idx', 1, 'has_pose', false),
        struct('name', 'UGV_PR', ...      % 加工区小车
               'goal_topic', '/TB3_2/goal_pose', ...
               'pose_topic', '/TB3_2/amcl_pose', ...
               'waypoints', [10, 5; 10, -5; 20, -5; 20, 5], ...
               'threshold', 0.2, 'current_idx', 1, 'has_pose', false)
    };
    for i = 1:length(robot_defs)
        robot_defs{i}.goal_pub = ros2publisher(node, robot_defs{i}.goal_topic, 'geometry_msgs/PoseStamped');
        robot_defs{i}.pose_sub = ros2subscriber(node, robot_defs{i}.pose_topic, 'geometry_msgs/PoseWithCovarianceStamped', ...
                                                'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);
        robot_defs{i}.last_pose = [];
    end
    robots = robot_defs;
end

% ------------------------------------------------------------------------
function [latest_uav_pos, robots] = wait_for_initial_poses(localPosSub, robots, timeout)
    fprintf('等待获取无人机和小车初始位置...\n');
    start_time = tic;
    latest_uav_pos = [NaN, NaN, NaN];
    while true
        all_received = true;
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
            error('超时：部分机器人未获取到位置。');
        end
        pause(0.2);
    end
end

% ------------------------------------------------------------------------
function publish_uav_trajectory(offboard_pub, traj_pub, vehicle_pub, ...
        uav_state, current_target, initial_target, target1, target2, ...
        setpoint_counter, timestamp)
    offboard_msg = ros2message(offboard_pub);
    offboard_msg.position     = true;
    offboard_msg.velocity     = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude     = false;
    offboard_msg.body_rate    = false;
    offboard_msg.timestamp    = timestamp;
    send(offboard_pub, offboard_msg);

    traj_msg = ros2message(traj_pub);
    switch uav_state
        case 'initial_climb', target = initial_target;
        case 'going_to_1',    target = target1;
        case 'going_to_2',    target = target2;
        otherwise,            target = current_target;
    end
    traj_msg.position     = single(target);
    traj_msg.velocity     = single([0,0,0]);
    traj_msg.acceleration = single([0,0,0]);
    traj_msg.jerk         = single([0,0,0]);
    traj_msg.yaw          = single(-3.14);
    traj_msg.yawspeed     = single(0.0);
    traj_msg.timestamp    = timestamp;
    send(traj_pub, traj_msg);

    if setpoint_counter == 10
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
function latest_uav_pos = get_uav_position(localPosSub, latest_uav_pos)
    try
        pos_msg = receive(localPosSub, 0.01);
        if ~isempty(pos_msg)
            latest_uav_pos = double([pos_msg.x, pos_msg.y, pos_msg.z]);
        end
    catch
    end
end

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
function [uav_state, current_target, wait_start_time, round_trip_count] = ...
        update_uav_state(uav_state, latest_uav_pos, current_target, ...
        initial_target, target1, target2, wait_start_time, round_trip_count)
    if strcmp(uav_state, 'finished') || any(isnan(latest_uav_pos))
        return;
    end
    switch uav_state
        case 'initial_climb'
            if abs(latest_uav_pos(1)-initial_target(1))<1.0 && ...
               abs(latest_uav_pos(2)-initial_target(2))<1.0 && ...
               abs(latest_uav_pos(3)-initial_target(3))<0.5
                uav_state = 'going_to_1';
                current_target = target1;
                disp('Reached initial climb point. Now flying to RM1.');
            end
        case 'going_to_1'
            if abs(latest_uav_pos(1)-target1(1))<0.1 && ...
               abs(latest_uav_pos(2)-target1(2))<0.1 && ...
               abs(latest_uav_pos(3)-target1(3))<0.1
                uav_state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target1;
                disp('Reached RM1. Waiting for 10 seconds...');
            end
        case 'going_to_2'
            if abs(latest_uav_pos(1)-target2(1))<0.1 && ...
               abs(latest_uav_pos(2)-target2(2))<0.1 && ...
               abs(latest_uav_pos(3)-target2(3))<0.1
                uav_state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target2;
                disp('Reached RM2. Waiting for 10 seconds...');
            end
        case 'waiting'
            if posixtime(datetime('now')) - wait_start_time >= 10.0   % 悬停10秒
                if isequal(current_target, target1)
                    uav_state = 'going_to_2';
                    disp('Wait finished. Flying to RM2.');
                elseif isequal(current_target, target2)
                    round_trip_count = round_trip_count + 1;
                    fprintf('Round trip %d completed.\n', round_trip_count);
                    if round_trip_count >= 10
                        uav_state = 'finished';
                        disp('Completed 10 round trips. Stopping.');
                    else
                        uav_state = 'going_to_1';
                        disp('Wait finished. Returning to RM1.');
                    end
                end
            end
    end
end

% ------------------------------------------------------------------------
function robots = update_and_send_robot_goals(robots, node)
    for i = 1:length(robots)
        robot = robots{i};
        if ~robot.has_pose; continue; end
        current_x = robot.last_pose.pose.pose.position.x;
        current_y = robot.last_pose.pose.pose.position.y;
        waypoint = robot.waypoints(robot.current_idx, :);
        dist = sqrt((current_x-waypoint(1))^2 + (current_y-waypoint(2))^2);
        if dist < robot.threshold
            fprintf('%s 已到达目标点 %d: (%.2f, %.2f)\n', robot.name, robot.current_idx, waypoint(1), waypoint(2));
            robot.current_idx = robot.current_idx + 1;
            if robot.current_idx > size(robot.waypoints,1)
                robot.current_idx = 1;
            end
            waypoint = robot.waypoints(robot.current_idx, :);
            fprintf('%s 开始前往下一个目标点 %d: (%.2f, %.2f)\n', robot.name, robot.current_idx, waypoint(1), waypoint(2));
        end
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
function print_status(uav_state, latest_uav_pos, current_target, robots)
    fprintf('\n===== 当前状态 (检查时刻) =====\n');
    if ~any(isnan(latest_uav_pos))
        fprintf('无人机 | 状态: %s | 位置: [%.2f, %.2f, %.2f] | 目标: [%.2f, %.2f, %.2f]\n', ...
                uav_state, latest_uav_pos(1), latest_uav_pos(2), latest_uav_pos(3), ...
                current_target(1), current_target(2), current_target(3));
    else
        fprintf('无人机 | 状态: %s | 等待位置数据...\n', uav_state);
    end
    for i = 1:length(robots)
        robot = robots{i};
        if robot.has_pose
            current_x = robot.last_pose.pose.pose.position.x;
            current_y = robot.last_pose.pose.pose.position.y;
            waypoint = robot.waypoints(robot.current_idx, :);
            dist = sqrt((current_x-waypoint(1))^2 + (current_y-waypoint(2))^2);
            fprintf('%s | 位置: [%.2f, %.2f] | 当前目标点 %d: [%.2f, %.2f] | 距离: %.2f\n', ...
                    robot.name, current_x, current_y, robot.current_idx, waypoint(1), waypoint(2), dist);
        else
            fprintf('%s | 等待位置数据...\n', robot.name);
        end
    end
    fprintf('================================\n\n');
end

% ==================== 通信链路计算函数 ====================
% 直连链路：UAV1 -> UAV2 (加工区基站)
function [SINR_dB, BER, delay_s] = compute_direct_link(uav1_world, uav2_world)
    % 直连使用加工区参数（因为 UAV2 位于加工区）
    Pt_dBm = 20;          % UAV1 发射功率
    PL0_dB = 40;
    c = 3e8;
    hop_delay = 0.001;
    
    % 加工区参数
    n = 2.5 + 0.5 * rand();          % 3.5~4.5
    noise_rise = 6 + 3 * rand();    % 12~15 dB
    N0_dBm = -100 + noise_rise;
    is_blocked = true;               % 加工区假设无额外遮挡（或根据实际）
    if is_blocked
        n_eff = n + 0.5;
    else
        n_eff = n;
    end
    
    [SINR_lin, SINR_dB, ~, ~, ~, d] = calc_single_link_deterministic(uav1_world, uav2_world, ...
        Pt_dBm, PL0_dB, n_eff, N0_dBm);
    BER = 0.5 * exp(-SINR_lin);
    delay_s = d / c + hop_delay;
end

% 中继链路：UAV1 -> UGV -> UAV2
% type: 'raw' 表示原材料区（第一跳使用原材料区参数，第二跳使用加工区参数）
%       'proc'表示加工区（两跳均使用加工区参数）
function [SINR_eq_dB, BER_eq, delay_s] = compute_relay_link(uav1_world, ugv_pos, uav2_world, type)
    Pt_uav_dBm = 20;
    Pt_ugv_dBm = 20;       % 小车发射功率 20 dBm
    PL0_dB = 40;
    c = 3e8;
    hop_delay = 0.001;     % 每跳1ms
    
    % 根据 type 选择参数
    if strcmp(type, 'raw')
        % 第一跳：原材料区（UAV1 -> UGV）
        n1 = 2.5 + 0.5 * rand();
        noise_rise1 = 6 + 3 * rand();
        N0_1_dBm = -100 + noise_rise1;
        is_blocked1 = true;   % 原材料区有遮挡
        if is_blocked1, n_eff1 = n1 + 0.5; else, n_eff1 = n1; end
        
        % 第二跳：加工区（UGV -> UAV2）
        n2 = 2.5 + 0.5 * rand();
        noise_rise2 = 6 + 3 * rand();
        N0_2_dBm = -100 + noise_rise2;
        is_blocked2 = true;
        if is_blocked2, n_eff2 = n2 + 0.5; else, n_eff2 = n2; end
    else  % 'proc' 加工区中继（UGV2 在加工区，两跳均在加工区）
        n1 = 2.5 + 0.5 * rand();
        noise_rise1 = 6 + 3 * rand();
        N0_1_dBm = -100 + noise_rise1;
        is_blocked1 = true;
        if is_blocked1, n_eff1 = n1 + 0.5; else, n_eff1 = n1; end
        
        n2 = 3.5 + 1.0 * rand();
        noise_rise2 = 12 + 3 * rand();
        N0_2_dBm = -100 + noise_rise2;
        is_blocked2 = false;
        if is_blocked2, n_eff2 = n2 + 0.5; else, n_eff2 = n2; end
    end
    
    % 第一跳
    [SINR1_lin, ~, ~, ~, ~, d1] = calc_single_link_deterministic(uav1_world, ugv_pos, ...
        Pt_uav_dBm, PL0_dB, n_eff1, N0_1_dBm);
    % 第二跳
    [SINR2_lin, ~, ~, ~, ~, d2] = calc_single_link_deterministic(ugv_pos, uav2_world, ...
        Pt_ugv_dBm, PL0_dB, n_eff2, N0_2_dBm);
    
    SINR_eq_lin = min(SINR1_lin, SINR2_lin);   % 解码转发
    SINR_eq_dB = 10 * log10(max(SINR_eq_lin, 1e-10));
    BER_eq = 0.5 * exp(-SINR_eq_lin);
    delay_s = (d1 + d2) / c + 2 * hop_delay;
end

% ------------------------------------------------------------------------
% 辅助函数：单链路计算（确定性小尺度衰落）
function [SINR_linear, SINR_dB, Pr_dBm, PL_dB, fade, d] = ...
        calc_single_link_deterministic(tx_pos, rx_pos, Pt_dBm, PL0_dB, n_eff, N0_dBm)
    d = norm(tx_pos - rx_pos);
    if d < 1, d = 1; end
    PL_dB = PL0_dB + 10 * n_eff * log10(d);
    
    fc = 2.4e9;
    lambda = 3e8 / fc;
    A = 0.1;
    fade = sqrt(1 + A * sin(2*pi*d / lambda));
    
    Pt_linear = 10^((Pt_dBm - 30)/10);
    PL_linear = 10^(-PL_dB / 10);
    Pr_linear = Pt_linear * PL_linear * fade^2;
    Pr_dBm = 10 * log10(Pr_linear) + 30;
    
    N0_linear = 10^((N0_dBm - 30)/10);
    SINR_linear = Pr_linear / N0_linear;
    SINR_dB = 10 * log10(max(SINR_linear, 1e-10));
end

% ------------------------------------------------------------------------
% 保存动态中继数据到 Excel
function save_dynamic_relay_data(data_log)
    if isempty(data_log)
        return;
    end
    colNames = {'Time_s', 'Selected_Link', 'SINR_dB', 'BER', 'Delay_s', ...
                'SINR_direct', 'SINR_ugv1', 'SINR_ugv2'};
    T = cell2table(data_log, 'VariableNames', colNames);
    filename = 'test.xlsx';
    try
        writetable(T, filename);
        fprintf('动态中继数据已保存到 %s (共 %d 条记录)\n', filename, height(T));
    catch ME
        fprintf('保存 Excel 文件失败: %s\n', ME.message);
    end
end