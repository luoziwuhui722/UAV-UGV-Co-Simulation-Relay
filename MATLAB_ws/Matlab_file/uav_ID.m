node = ros2node("sysid_check");

% 订阅 vehicle_status_v1 话题
sub = ros2subscriber(node, "fmu/out/vehicle_status_v1", "px4_msgs/VehicleStatus", ...
    "Reliability", "besteffort");
sub1 = ros2subscriber(node, "/px4_1/fmu/out/vehicle_status_v1", "px4_msgs/VehicleStatus", ...
    "Reliability", "besteffort");
sub2 = ros2subscriber(node, "/px4_2/fmu/out/vehicle_status_v1", "px4_msgs/VehicleStatus", ...
    "Reliability", "besteffort");

% 接收一条消息（等待最多 2 秒）
try
    msg = receive(sub, 2);
    fprintf("无人机系统ID (system_id) = %d\n", msg.system_id);
    msg1 = receive(sub1, 2);
    fprintf("UAV1无人机系统ID (system_id) = %d\n", msg1.system_id);
    msg2 = receive(sub2, 2);
    fprintf("UAV2无人机系统ID (system_id) = %d\n", msg2.system_id);
catch
    disp("未在超时时间内收到消息，请检查话题是否存在或无人机是否正在运行。");
end