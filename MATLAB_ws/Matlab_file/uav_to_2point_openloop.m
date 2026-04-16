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
    
    % 根据 setpoint_counter 切换目标点（10秒 = 100次循环）
    if setpoint_counter < 100   % 前10秒发送第一个目标点
        target_position = single([-10.0, -5.0, -3.0]);  % 目标点1
    else                        % 10秒后发送第二个目标点
        target_position = single([-20.0, 5.0, -3.0]);   % 目标点2
    end
    
    traj_msg.position     = target_position;
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
    
    %% 新增：获取并打印当前位置与目标位置
    % 非阻塞接收位置消息（超时0.01秒，避免影响发布周期）
    try
        pos_msg = receive(localPosSub, 0.01);
        if ~isempty(pos_msg)
            % 位置坐标：x, y, z (NED坐标系，z向下为正)
            latest_pos = double([pos_msg.x, pos_msg.y, pos_msg.z]);
        end
    catch
        % 无新消息则忽略
    end
    
    % 打印（仅当已收到有效位置时）
    if ~any(isnan(latest_pos))
        fprintf('Current position: [%.2f, %.2f, %.2f] | Target position: [%.2f, %.2f, %.2f]\n', ...
                latest_pos(1), latest_pos(2), latest_pos(3), ...
                target_position(1), target_position(2), target_position(3));
    else
        fprintf('Waiting for position data... Target: [%.2f, %.2f, %.2f]\n', ...
                target_position(1), target_position(2), target_position(3));
    end
    
    %% 4. Increment counter
    setpoint_counter = setpoint_counter + 1;
    
    pause(publish_rate); % 100 ms
end