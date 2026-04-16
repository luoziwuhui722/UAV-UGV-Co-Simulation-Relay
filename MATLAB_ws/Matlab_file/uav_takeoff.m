%% OffboardControlSimulation_PX4_Infinite.m
clc; clear; close all;

publish_rate = 0.1; % 100 ms
setpoint_counter = 0;

%% ROS2 Setup
ros2node = ros2node("offboard_control_node");

offboard_pub = ros2publisher(ros2node, "/fmu/in/offboard_control_mode", "px4_msgs/OffboardControlMode");
traj_pub     = ros2publisher(ros2node, "/fmu/in/trajectory_setpoint", "px4_msgs/TrajectorySetpoint");
vehicle_pub  = ros2publisher(ros2node, "/fmu/in/vehicle_command", "px4_msgs/VehicleCommand");

disp("Starting Offboard control simulation... (Ctrl+C to stop)");

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
    traj_msg.position     = single([-10.0, 0.0, -5.0]);  % z = -5 meters
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
    
    %% 4. Increment counter
    setpoint_counter = setpoint_counter + 1;
    
    pause(publish_rate); % 100 ms
end