function set_pose(botName, x, y, theta)
    % Initialize ROS 2
    nodeName = ['/matlab_turtlebot_control_' int2str(1000000000*rand(1))]; % Node Name
    ctrlNode = ros2node(nodeName);

    % Create the turtlebot control topics
    botTopic = sprintf('/%s/set_pose', botName);
    [botPublisher, ~] = ros2publisher(ctrlNode, botTopic, "geometry_msgs/PoseStamped", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');

    pose = ros2message('geometry_msgs/PoseStamped');
    pose.pose.position.x = x;
    pose.pose.position.y = y;

    Qwxyz = angle2quat(theta, 0.0, 0.0, "ZYX");
    pose.pose.orientation.w = Qwxyz(1);
    pose.pose.orientation.x = Qwxyz(2);
    pose.pose.orientation.y = Qwxyz(3);
    pose.pose.orientation.z = Qwxyz(4);

    pause(0.1);

    send(botPublisher, pose);
end
