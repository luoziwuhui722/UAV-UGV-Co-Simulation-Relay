%% ==================== 主函数 ====================
% 主函数入口：执行无人机与多台移动机器人的协同控制
% 固定中继场景：UAV1 → UGV2 → UAV2
% UAV1 发射功率 20 dBm，UGV2 发射功率 25 dBm，UAV2 高度 3 米
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

%% 数据记录初始化
data_log = {};                        % 存储每条记录的 cell 数组
start_clock = tic;                    % 相对时间计时器（秒）

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

        % ---------- 固定中继（UGV2）通信指标 ----------
        % 转换无人机坐标到世界坐标系
        uav1_world_pos = [latest_uav_pos(2), latest_uav_pos(1), -latest_uav_pos(3)];
        uav2_pos = [15, 0, 2];                     % 基站位置（高度改为3米）
        if robots{2}.has_pose                      % UGV2 对应 robots{2}
            ugv2_pos = [robots{2}.last_pose.pose.pose.position.x, ...
                        robots{2}.last_pose.pose.pose.position.y, 0];
            % 调用函数计算所有指标（打印+返回数值）
            [d1, PL1_dB, Pr1_dBm, SINR1_dB, BER1, ...
             d2, PL2_dB, Pr2_dBm, SINR2_dB, BER2, ...
             SINR_eq_dB, BER_eq, delay_s] = compute_fixed_relay_ugv2_metrics(uav1_world_pos, ugv2_pos, uav2_pos);
            
            % 记录数据
            elapsed_time = toc(start_clock);
            data_row = {elapsed_time, ...
                d1, PL1_dB, Pr1_dBm, SINR1_dB, BER1, ...
                d2, PL2_dB, Pr2_dBm, SINR2_dB, BER2, ...
                SINR_eq_dB, BER_eq, delay_s};
            data_log = [data_log; data_row];
            
            % 立即保存到 Excel（每次覆盖，保证数据不丢失）
            save_fixed_relay_ugv2_data_to_excel(data_log);
        else
            fprintf('【固定中继(UGV2)】UGV2 位置尚未获取，跳过计算\n');
        end
        % -------------------------------------------------

        % 重置计时器
        last_check_time = tic;
    end

    setpoint_counter = setpoint_counter + 1;
    pause(publish_rate);
end






%% ==================== 子函数 ====================
% ------------------------------------------------------------------------
% 函数名称: init_ros2
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
% 函数名称: init_robots
% ------------------------------------------------------------------------
function robots = init_robots(node)
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
    for i = 1:length(robot_defs)
        robot_defs{i}.goal_pub = ros2publisher(node, robot_defs{i}.goal_topic, 'geometry_msgs/PoseStamped');
        robot_defs{i}.pose_sub = ros2subscriber(node, robot_defs{i}.pose_topic, 'geometry_msgs/PoseWithCovarianceStamped', ...
                                                'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);
        robot_defs{i}.last_pose = [];
    end
    robots = robot_defs;
end

% ------------------------------------------------------------------------
% 函数名称: wait_for_initial_poses
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
% 函数名称: publish_uav_trajectory
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
        case 'initial_climb'
            target = initial_target;
        case 'going_to_1'
            target = target1;
        case 'going_to_2'
            target = target2;
        case {'waiting', 'finished'}
            target = current_target;
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
% 函数名称: get_uav_position
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
% 函数名称: get_robot_poses
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
                disp('Reached initial climb point. Now flying to target point 1.');
            end
        case 'going_to_1'
            if abs(latest_uav_pos(1)-target1(1))<0.1 && ...
               abs(latest_uav_pos(2)-target1(2))<0.1 && ...
               abs(latest_uav_pos(3)-target1(3))<0.1
                uav_state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target1;
                disp('Reached target point 1. Waiting for 10 seconds...');
            end
        case 'going_to_2'
            if abs(latest_uav_pos(1)-target2(1))<0.1 && ...
               abs(latest_uav_pos(2)-target2(2))<0.1 && ...
               abs(latest_uav_pos(3)-target2(3))<0.1
                uav_state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target2;
                disp('Reached target point 2. Waiting for 10 seconds...');
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
                        disp('Completed 10 round trips. Stopping.');
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
% 函数名称: print_status
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

% ==================== 固定中继（UGV2）通信指标计算 ====================

% ------------------------------------------------------------------------
% 函数名称: compute_fixed_relay_ugv2_metrics
% ------------------------------------------------------------------------
function [d1, PL1_dB, Pr1_dBm, SINR1_dB, BER1, ...
          d2, PL2_dB, Pr2_dBm, SINR2_dB, BER2, ...
          SINR_eq_dB, BER_eq, delay_s] = compute_fixed_relay_ugv2_metrics(uav1_pos, ugv2_pos, uav2_pos)
    % 固定系统参数
    Pt_uav_dBm = 20;        % 无人机发射功率 20 dBm
    Pt_ugv_dBm = 20;        % 小车发射功率 25 dBm
    PL0_dB = 40;            % 1米处路径损耗 dB
    c = 3e8;                % 光速 m/s
    hop_delay = 0.001;      % 每跳处理延迟 1 ms

    if any(isnan(uav1_pos)) || any(isinf(uav1_pos))
        error('UAV1 位置无效');
    end
    if any(isnan(ugv2_pos)) || any(isinf(ugv2_pos))
        error('UGV2 位置无效');
    end

    % 第一跳：UAV1 → UGV2（原材料区）
    n_raw = 2.5 + 0.5 * rand();
    noise_rise_raw = 6 + 3 * rand();
    N0_raw_dBm = -100 + noise_rise_raw;
    m_raw = 1.5 - 0.75 * rand();   % 保留但不再使用
    is_blocked_raw = true;
    if is_blocked_raw
        n_eff_raw = n_raw + 0.5;
    else
        n_eff_raw = n_raw;
    end
    [SINR1_lin, SINR1_dB, Pr1_dBm, PL1_dB, ~, d1] = ...
        calc_single_link_deterministic(uav1_pos, ugv2_pos, Pt_uav_dBm, PL0_dB, n_eff_raw, N0_raw_dBm);
    BER1 = 0.5 * exp(-SINR1_lin);

    % 第二跳：UGV2 → UAV2（加工区）
    n_proc = 3.5 + 1.0 * rand();
    noise_rise_proc = 12 + 3 * rand();
    N0_proc_dBm = -100 + noise_rise_proc;
    m_proc = 1.0 - 0.5 * rand();   % 保留但不再使用
    is_blocked_proc = false;
    if is_blocked_proc
        n_eff_proc = n_proc + 0.5;
    else
        n_eff_proc = n_proc;
    end
    [SINR2_lin, SINR2_dB, Pr2_dBm, PL2_dB, ~, d2] = ...
        calc_single_link_deterministic(ugv2_pos, uav2_pos, Pt_ugv_dBm, PL0_dB, n_eff_proc, N0_proc_dBm);
    BER2 = 0.5 * exp(-SINR2_lin);

    SINR_eq_lin = min(SINR1_lin, SINR2_lin);
    SINR_eq_dB = 10 * log10(max(SINR_eq_lin, 1e-10));
    BER_eq = 0.5 * exp(-SINR_eq_lin);
    delay_s = (d1 + d2) / c + 2 * hop_delay;

    fprintf('【固定中继通信指标】(UAV1→UGV2→UAV2, UGV2在加工区)\n');
    fprintf('=== 第一跳 (UAV1 → UGV2) 原材料区 (UAV Tx=20dBm) ===\n');
    fprintf('  距离: %.2f m | 路径损耗: %.2f dB | 接收功率: %.2f dBm | SINR: %.2f dB | BER: %.2e\n', ...
            d1, PL1_dB, Pr1_dBm, SINR1_dB, BER1);
    fprintf('=== 第二跳 (UGV2 → UAV2) 加工区 (UGV Tx=25dBm) ===\n');
    fprintf('  距离: %.2f m | 路径损耗: %.2f dB | 接收功率: %.2f dBm | SINR: %.2f dB | BER: %.2e\n', ...
            d2, PL2_dB, Pr2_dBm, SINR2_dB, BER2);
    fprintf('=== 端到端等效 ===\n');
    fprintf('  等效 SINR: %.2f dB | 等效 BER: %.2e | 延迟: %.3f ms\n', ...
            SINR_eq_dB, BER_eq, delay_s*1000);
    fprintf('  -------------------------------------------------\n');
end

% ------------------------------------------------------------------------
% 辅助函数: calc_single_link_deterministic
% 功能描述: 使用确定性小尺度衰落（固定为1），仅由大尺度路径损耗决定
% ------------------------------------------------------------------------
function [SINR_linear, SINR_dB, Pr_dBm, PL_dB, fade, d] = ...
        calc_single_link_deterministic(tx_pos, rx_pos, Pt_dBm, PL0_dB, n_eff, N0_dBm)
    d = norm(tx_pos - rx_pos);
    if d < 1, d = 1; end
    PL_dB = PL0_dB + 10 * n_eff * log10(d);
    
    % 小尺度衰落因子：基于距离的确定性正弦函数（模拟空间多径起伏）
    fc = 2.4e9;          % 载波频率 2.4 GHz
    lambda = 3e8 / fc;   % 波长 ≈ 0.125 m
    A = 0.1;             % 衰落深度 (0 < A < 1)
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
% 辅助函数: nakagami_fade (保留但不再使用)
% ------------------------------------------------------------------------
function fade = nakagami_fade(m)
    if m < 0.01, m = 0.01; end
    fade = sqrt(gamrnd(m, 1/m));
end

% ------------------------------------------------------------------------
% 函数名称: save_fixed_relay_ugv2_data_to_excel
% ------------------------------------------------------------------------
function save_fixed_relay_ugv2_data_to_excel(data_log)
    if isempty(data_log)
        return;
    end
    colNames = {'Time_s', ...
        'Hop1_Distance_m', 'Hop1_PathLoss_dB', 'Hop1_RxPower_dBm', 'Hop1_SINR_dB', 'Hop1_BER', ...
        'Hop2_Distance_m', 'Hop2_PathLoss_dB', 'Hop2_RxPower_dBm', 'Hop2_SINR_dB', 'Hop2_BER', ...
        'Eq_SINR_dB', 'Eq_BER', 'Eq_Delay_s'};
    T = cell2table(data_log, 'VariableNames', colNames);
    filename = 'fixed_relay_ugv2_data.xlsx';
    try
        writetable(T, filename);
        fprintf('数据已保存到 %s (共 %d 条记录)\n', filename, height(T));
    catch ME
        fprintf('保存 Excel 文件失败: %s\n', ME.message);
    end
end