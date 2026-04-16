%% 环境设置
setenv('ROS_DOMAIN_ID', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

%% 定义机器人信息
robots = {
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

%% 创建 ROS2 节点
node = ros2node("/matlab_waypoint_nav");

%% 为每个机器人创建发布器和订阅器
for i = 1:length(robots)
    robots{i}.goal_pub = ros2publisher(node, robots{i}.goal_topic, 'geometry_msgs/PoseStamped');
    robots{i}.pose_sub = ros2subscriber(node, robots{i}.pose_topic, 'geometry_msgs/PoseWithCovarianceStamped', ...
                                        'Reliability', 'reliable', 'History', 'keeplast', 'Depth', 10);
end

%% 等待所有机器人获取初始位置
fprintf('等待获取所有机器人初始位置...\n');
timeout = 10;  % 最多等待10秒
start_time = tic;
while true
    all_received = true;
    for i = 1:length(robots)
        pose_msg = robots{i}.pose_sub.LatestMessage;
        if ~isempty(pose_msg)
            robots{i}.has_pose = true;
            robots{i}.last_pose = pose_msg;
            fprintf('成功获取 %s 位置！当前位置 (%.2f, %.2f)\n', ...
                    robots{i}.name, pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
        else
            all_received = false;
        end
    end
    if all_received
        fprintf('所有机器人位置已获取！\n');
        break;
    end
    if toc(start_time) > timeout
        error('超时：部分机器人未获取到位置。请检查 AMCL 是否运行正常或是否已在 RViz 中设置初始位姿。');
    end
    pause(0.2);
end

%% 主控制循环
fprintf('开始多车导航...\n');
while true
    for i = 1:length(robots)
        robot = robots{i};
        
        % 获取最新定位消息
        pose_msg = robot.pose_sub.LatestMessage;
        if isempty(pose_msg)
            % 若暂时未收到定位，跳过本次控制，等待下一次循环
            fprintf('未收到 %s 的定位消息，跳过本次控制\n', robot.name);
            continue;
        end
        robot.last_pose = pose_msg;
        current_x = pose_msg.pose.pose.position.x;
        current_y = pose_msg.pose.pose.position.y;
        
        % 当前目标点
        waypoint = robot.waypoints(robot.current_idx, :);
        dist = sqrt((current_x - waypoint(1))^2 + (current_y - waypoint(2))^2);
        
        % 判断是否到达
        if dist < robot.threshold
            fprintf('%s 已到达目标点 %d: (%.2f, %.2f), 距离 %.2f\n', ...
                    robot.name, robot.current_idx, waypoint(1), waypoint(2), dist);
            
            % 切换到下一个目标点
            robot.current_idx = robot.current_idx + 1;
            if robot.current_idx > size(robot.waypoints, 1)
                robot.current_idx = 1;  % 循环
            end
            waypoint = robot.waypoints(robot.current_idx, :);
            fprintf('%s 开始前往下一个目标点 %d: (%.2f, %.2f)\n', ...
                    robot.name, robot.current_idx, waypoint(1), waypoint(2));
            
            % 发送新目标点
            goal_msg = ros2message(robot.goal_pub);
            goal_msg.header.stamp = ros2time(node, "now");
            goal_msg.header.frame_id = 'map';
            goal_msg.pose.position.x = waypoint(1);
            goal_msg.pose.position.y = waypoint(2);
            goal_msg.pose.position.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;
            send(robot.goal_pub, goal_msg);
            fprintf('%s 已发送新目标点 %d: (%.2f, %.2f)\n', robot.name, robot.current_idx, waypoint(1), waypoint(2));
        else
            % 未到达，重发当前目标点
            goal_msg = ros2message(robot.goal_pub);
            goal_msg.header.stamp = ros2time(node, "now");
            goal_msg.header.frame_id = 'map';
            goal_msg.pose.position.x = waypoint(1);
            goal_msg.pose.position.y = waypoint(2);
            goal_msg.pose.position.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;
            send(robot.goal_pub, goal_msg);
            fprintf('%s 距离目标点 %d 还有 %.2f 米，重发指令...\n', ...
                    robot.name, robot.current_idx, dist);
        end
        
        % 保存修改后的机器人状态
        robots{i} = robot;
    end
    
    % 每 0.5 秒检查一次
    pause(1);
end

% 注意：按 Ctrl+C 可停止脚本