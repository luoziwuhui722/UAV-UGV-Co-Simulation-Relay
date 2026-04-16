% 初始化 ROS 2 节点
node = ros2node("/matlab_subscriber");

% 创建订阅者，指定话题和消息类型
sub = ros2subscriber(node, "fmu/out/vehicle_local_position_v1", "px4_msgs/VehicleLocalPosition", ...
    "Reliability", "besteffort");

% 定义回调函数，每次收到消息时打印位置
function callback(msg)
    % 提取位置 (x, y, z)
    pos = [double(msg.x), double(msg.y), double(msg.z)];
    fprintf("位置: x = %.3f, y = %.3f, z = %.3f\n", pos(1), pos(2), pos(3));
end

% 将回调函数绑定到订阅者
sub.NewMessageFcn = @callback;

% 保持节点运行，等待消息
disp("等待无人机位置消息... (按 Ctrl+C 停止)");
while true
    pause(0.1);  % 保持循环
end