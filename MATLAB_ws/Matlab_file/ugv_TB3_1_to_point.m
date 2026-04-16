%% 设置环境
setenv('ROS_DOMAIN_ID', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

%% 创建节点和发布者
node = ros2node("/matlab_goal_publisher");
goal_pub = ros2publisher(node, '/TB3_1/goal_pose', 'geometry_msgs/PoseStamped');

%% 构造目标点消息
goal_msg = ros2message(goal_pub);
goal_msg.header.stamp = ros2time(node, "now");
goal_msg.header.frame_id = 'map';
goal_msg.pose.position.x = 1.0;
goal_msg.pose.position.y = 1.0;
goal_msg.pose.position.z = 0.0;
goal_msg.pose.orientation.w = 1.0;  % 默认朝向

%% 发送
send(goal_pub, goal_msg);
disp('目标点 (1, 1) 已发送给 TB3_1');