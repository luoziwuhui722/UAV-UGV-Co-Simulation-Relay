%% OffboardControlSimulation_PX4_Infinite.m
clc; clear; close all;

publish_rate = 0.1; % 100 ms
setpoint_counter = 0;

%% ROS2 Setup
ros2node = ros2node("offboard_control_node");

offboard_pub = ros2publisher(ros2node, "/fmu/in/offboard_control_mode", "px4_msgs/OffboardControlMode");
traj_pub     = ros2publisher(ros2node, "/fmu/in/trajectory_setpoint", "px4_msgs/TrajectorySetpoint");
vehicle_pub  = ros2publisher(ros2node, "/fmu/in/vehicle_command", "px4_msgs/VehicleCommand");

% 修正订阅：使用正确的节点名，并订阅位置信息（VehicleLocalPosition）
vehicleStatusSub = ros2subscriber(ros2node, "/fmu/out/vehicle_status_v1", "px4_msgs/VehicleStatus", "Reliability", "besteffort");
localPosSub      = ros2subscriber(ros2node, "/fmu/out/vehicle_local_position_v1", "px4_msgs/VehicleLocalPosition", "Reliability", "besteffort");

disp("Starting Offboard control simulation... (Ctrl+C to stop)");

% 用于存储最新位置
latest_pos = [NaN, NaN, NaN];

% 状态机定义
% state: 'going_to_1'  -> 飞向目标点1
%        'going_to_2'  -> 飞向目标点2
%        'waiting'     -> 停留中
%        'finished'    -> 已完成10次来回，停止移动
state = 'going_to_1';           % 初始状态：飞向目标点1
wait_start_time = 0;            % 停留开始时间（Unix 秒）
round_trip_count = 0;           % 已完成的来回次数（一次来回 = 1->2->1）

% 目标点坐标
target1 = [-5.0, -10.0, -5.0];
target2 = [5.0, -20.0, -5.0];
current_target = target1;       % 当前目标点（用于发布）

while true
    timestamp = uint64(posixtime(datetime('now'))*1e6);
    
    %% 1. Publish OffboardControlMode
    offboard_msg = ros2message(offboard_pub);
    offboard_msg.position     = true;
    offboard_msg.velocity     = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude     = false;
    offboard_msg.body_rate    = false;
    offboard_msg.timestamp    = timestamp;
    send(offboard_pub, offboard_msg);
    
    %% 2. Publish TrajectorySetpoint
    traj_msg = ros2message(traj_pub);
    
    % 根据状态选择当前目标点
    if strcmp(state, 'going_to_1')
        current_target = target1;
    elseif strcmp(state, 'going_to_2')
        current_target = target2;
    elseif strcmp(state, 'waiting')
        % 停留期间保持当前目标点不变（即刚到达的位置）
        % current_target 已经在进入 waiting 状态时设置好
    elseif strcmp(state, 'finished')
        % 完成后停在最后一个目标点（目标点1）
        current_target = target1;
    end
    
    traj_msg.position     = single(current_target);
    traj_msg.velocity     = single([0.0, 0.0, 0.0]);
    traj_msg.acceleration = single([0.0, 0.0, 0.0]);
    traj_msg.jerk         = single([0.0, 0.0, 0.0]);
    traj_msg.yaw          = single(-3.14);
    traj_msg.yawspeed     = single(0.0);
    traj_msg.timestamp    = timestamp;
    send(traj_pub, traj_msg);
    
    %% 3. Switch to Offboard mode and Arm once
    if setpoint_counter == 10
        % Offboard mode
        mode_msg = ros2message(vehicle_pub);
        mode_msg.command          = uint32(176); % VEHICLE_CMD_DO_SET_MODE
        mode_msg.param1           = single(1);
        mode_msg.param2           = single(6);
        mode_msg.target_system    = uint8(1);
        mode_msg.target_component = uint8(1);
        mode_msg.source_system    = uint8(1);
        mode_msg.source_component = uint16(1);  % PX4 latest
        mode_msg.from_external    = true;
        mode_msg.timestamp        = timestamp;
        send(vehicle_pub, mode_msg);
        
        % Arm
        arm_msg = ros2message(vehicle_pub);
        arm_msg.command          = uint32(400); % VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_msg.param1           = single(1.0); % arm
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
    
    %% 获取当前位置并更新状态
    try
        pos_msg = receive(localPosSub, 0.01);
        if ~isempty(pos_msg)
            latest_pos = double([pos_msg.x, pos_msg.y, pos_msg.z]);
        end
    catch
        % 无新消息则忽略
    end
    
    % 如果已完成，不再进行状态切换
    if ~strcmp(state, 'finished') && ~any(isnan(latest_pos))
        % 到达检测
        if strcmp(state, 'going_to_1')
            dx = abs(latest_pos(1) - target1(1));
            dy = abs(latest_pos(2) - target1(2));
            dz = abs(latest_pos(3) - target1(3));
            if dx < 0.1 && dy < 0.1 && dz < 0.1
                % 到达目标点1，进入停留状态
                state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                % 停留期间目标点保持为 target1
                current_target = target1;
                disp('Reached target point 1. Waiting for 3 seconds...');
            end
        elseif strcmp(state, 'going_to_2')
            dx = abs(latest_pos(1) - target2(1));
            dy = abs(latest_pos(2) - target2(2));
            dz = abs(latest_pos(3) - target2(3));
            if dx < 0.1 && dy < 0.1 && dz < 0.1
                % 到达目标点2，进入停留状态
                state = 'waiting';
                wait_start_time = posixtime(datetime('now'));
                current_target = target2;
                disp('Reached target point 2. Waiting for 3 seconds...');
            end
        elseif strcmp(state, 'waiting')
            % 检查停留时间是否已过3秒
            if posixtime(datetime('now')) - wait_start_time >= 3.0
                % 停留结束，根据当前目标点切换下一个目标点
                if isequal(current_target, target1)
                    % 从目标点1离开，前往目标点2
                    state = 'going_to_2';
                    disp('Wait finished. Flying to target point 2.');
                elseif isequal(current_target, target2)
                    % 从目标点2离开，返回目标点1，并增加一次来回计数
                    round_trip_count = round_trip_count + 1;
                    fprintf('Round trip %d completed.\n', round_trip_count);
                    if round_trip_count >= 10
                        % 完成10次来回，进入完成状态，停在目标点1
                        state = 'finished';
                        disp('Completed 10 round trips. Stopping at target point 1.');
                    else
                        state = 'going_to_1';
                        disp('Wait finished. Returning to target point 1.');
                    end
                end
            end
        end
    end
    
    %% 打印当前位置与目标位置（含状态）
    if ~any(isnan(latest_pos))
        fprintf('State: %s | Current: [%.2f, %.2f, %.2f] | Target: [%.2f, %.2f, %.2f]\n', ...
                state, latest_pos(1), latest_pos(2), latest_pos(3), ...
                current_target(1), current_target(2), current_target(3));
    else
        fprintf('State: %s | Waiting for position data... Target: [%.2f, %.2f, %.2f]\n', ...
                state, current_target(1), current_target(2), current_target(3));
    end
    
    %% 4. Increment counter
    setpoint_counter = setpoint_counter + 1;
    
    pause(publish_rate); % 100 ms
end