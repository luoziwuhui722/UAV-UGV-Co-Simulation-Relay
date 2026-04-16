%% 环境设置
setenv('ROS_DOMAIN_ID', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

%% 创建节点、发布器和订阅器（指定 QoS）
node = ros2node("/matlab_waypoint_nav");
goal_pub = ros2publisher(node, '/TB3_1/goal_pose', 'geometry_msgs/PoseStamped');

% 订阅 amcl_pose，使用 reliable QoS
pose_sub = ros2subscriber(node, '/TB3_1/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped', ...
                          'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);

fprintf('等待获取机器人初始位置...\n');
timeout = 10;  % 最大等待时间（秒）
start_time = tic;
while toc(start_time) < timeout
    pose_msg = pose_sub.LatestMessage;
    if ~isempty(pose_msg)
        fprintf('成功获取位置！当前位置 (%.2f, %.2f)\n', ...
                pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
        break;
    end
    pause(0.2);
end
if isempty(pose_msg)
    error('无法获取机器人位置。请检查：\n1. AMCL 是否运行正常？\n2. 是否已在 RViz 中设置初始位姿？');
end

%% 定义四个目标点 (x, y)
waypoints = [ 1.0, 1.0;
             -1.0, 1.0;
             -1.0,-1.0;
              1.0,-1.0 ];

threshold = 0.2; % 到达判定阈值（米）

%% 循环发送目标点
for i = 1:size(waypoints,1)
    % 构造目标点消息
    goal_msg = ros2message(goal_pub);
    goal_msg.header.stamp = ros2time(node, "now");
    goal_msg.header.frame_id = 'map';
    goal_msg.pose.position.x = waypoints(i,1);
    goal_msg.pose.position.y = waypoints(i,2);
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.w = 1.0; % 默认朝向

    % 发送目标点
    send(goal_pub, goal_msg);
    fprintf('发送目标点 %d: (%.2f, %.2f)\n', i, waypoints(i,1), waypoints(i,2));

    % 等待到达该点
    while true
        pose_msg = pose_sub.LatestMessage;
        if ~isempty(pose_msg)
            current_x = pose_msg.pose.pose.position.x;
            current_y = pose_msg.pose.pose.position.y;
            dist = sqrt((current_x - waypoints(i,1))^2 + (current_y - waypoints(i,2))^2);
            if dist < threshold
                fprintf('已到达目标点 %d，距离 %.2f\n', i, dist);
                break;
            end
        end
        pause(0.5); % 避免过于频繁的检查
    end

    pause(1); % 到达后稍作停顿
end

disp('所有目标点已执行完毕！');