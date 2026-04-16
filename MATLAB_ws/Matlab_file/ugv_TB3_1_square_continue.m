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
waypoints = [ -10.0, 5.0;
             -10.0, -6.0;
             -20.0,-6.0;
              -20.0,6.0 ];

threshold = 0.2; % 到达判定阈值（米）

%% 无限循环，使小车一直做正方形运动
while true
    for i = 1:size(waypoints,1)
        arrived = false;          % 到达标志
        fprintf('开始前往目标点 %d: (%.2f, %.2f)\n', i, waypoints(i,1), waypoints(i,2));
        
        while ~arrived
            % 构造并发送当前目标点（每次都更新时间戳）
            goal_msg = ros2message(goal_pub);
            goal_msg.header.stamp = ros2time(node, "now");      % 使用当前时间
            goal_msg.header.frame_id = 'map';
            goal_msg.pose.position.x = waypoints(i,1);
            goal_msg.pose.position.y = waypoints(i,2);
            goal_msg.pose.position.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;
            
            send(goal_pub, goal_msg);
            
            % 获取最新位置并判断是否到达
            pose_msg = pose_sub.LatestMessage;
            if ~isempty(pose_msg)
                current_x = pose_msg.pose.pose.position.x;
                current_y = pose_msg.pose.pose.position.y;
                dist = sqrt((current_x - waypoints(i,1))^2 + (current_y - waypoints(i,2))^2);
                if dist < threshold
                    fprintf('已到达目标点 %d，距离 %.2f\n', i, dist);
                    arrived = true;   % 退出内层循环
                else
                    % 未到达，等待0.5秒后重发
                    fprintf('距离目标点 %d 还有 %.2f 米，0.5秒后重发...\n', i, dist);
                    pause(0.5);
                end
            else
                % 未收到定位消息，等待0.5秒后重发
                fprintf('未收到定位消息，0.5秒后重发目标点 %d...\n', i);
                pause(0.5);
            end
        end
        
        pause(1); % 到达后稍作停顿，再发下一个点
    end
    % 完成一圈后自动进入下一圈
end

% 注意：由于是无限循环，需要手动停止脚本（如按 Ctrl+C）