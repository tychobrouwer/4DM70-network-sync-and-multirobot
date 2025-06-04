% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE yOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   % Set ROS network settings
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID);
%   
%   demo_threeturtlebot_simulator()

function demo_threeturtlebot_simulator()
% Authors: Omur Arslan, o.arslan@tue.nl
% Created: June 02, 2023
% Modified: June 03, 2024 (Visualization is added)

    %% ROS Node Settings
    % Register the leader-follower node at the ROS master
    nodeName = ['/matlab_threeturtlebot_simulator' int2str(1000000000*rand(1))]; % Node Name
    simulatorNode = ros2node(nodeName);

    % Turtlebot 1
    threeturtlebot(1).name = 'turtlebot_red';
    threeturtlebot(1).color = [1 0 0]*0.8;
    threeturtlebot(1).pose =  core_ros2tools.ROS2MessageHandle(turtlebot_set_pose(1.0, 1.0, pi/2)); %[x(m), y(m), theta (rad)]
    threeturtlebot(1).velocity = core_ros2tools.ROS2MessageHandle(turtlebot_set_velocity(0.0, 0.0)); %[linear_velocity (m/s), angular_velocity(rad/s)]
    % Turtlebot 2
    threeturtlebot(2).name = 'turtlebot_green';
    threeturtlebot(2).color = [0 1 0]*0.8;
    threeturtlebot(2).pose = core_ros2tools.ROS2MessageHandle(turtlebot_set_pose(2.0, 1.0, pi/2)); % [x(m), y(m), theta (rad)]
    threeturtlebot(2).velocity = core_ros2tools.ROS2MessageHandle(turtlebot_set_velocity(0.0, 0.0)); %[linear_velocity (m/s), angular_velocity(rad/s)]

    % Turtlebot 3
    threeturtlebot(3).name = 'turtlebot_blue';
    threeturtlebot(3).color = [0 0 1]*0.8 + 0.2;     
    threeturtlebot(3).pose = core_ros2tools.ROS2MessageHandle(turtlebot_set_pose(3.0, 1.0, pi/2)); % [x(m), y(m), theta (rad)]
    threeturtlebot(3).velocity = core_ros2tools.ROS2MessageHandle(turtlebot_set_velocity(0.0, 0.0)); %[linear_velocity (m/s), angular_velocity(rad/s)]
    
    for k = 1:numel(threeturtlebot)
        threeturtlebot(k).poseTopic = sprintf('/mocap/%s/pose', threeturtlebot(k).name);
        % Create a turtlebot pose publisher
        posePublisher = ros2publisher(simulatorNode, threeturtlebot(k).poseTopic, "geometry_msgs/PoseStamped",...
            'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
        threeturtlebot(k).posePublisher = posePublisher;
        
        threeturtlebot(k).ctrlTopic = sprintf('/%s/cmd_vel_ctrl', threeturtlebot(k).name);
        % Create a turtlebot control subscriber
        threeturtlebot(k).ctrlSubscriber = ros2subscriber(simulatorNode, threeturtlebot(k).ctrlTopic, 'geometry_msgs/Twist',@(msg) turtlebot_ctrl_callback(msg, threeturtlebot(k)) , ...
            'History','keeplast','Depth', 1, 'Reliability','besteffort');

    end
    
    % Start Visualization
    figure; hold on; box on; grid on; axis equal;
    axis_handle = gca;
    for k = 1:numel(threeturtlebot)
        threeturtlebot(k).plot_handle = turtlebot_plot(threeturtlebot(k));
    end

    % Simulator Loop
    ctrl_rate = 20;
    while true
        % Update turtlebot states
        threeturtlebot = turtlebot_state_update(threeturtlebot, 1.0/ctrl_rate);

        % Publish TurtleBot poses
        for k = 1:numel(threeturtlebot)
            threeturtlebot(k).posePublisher.send(threeturtlebot(k).pose.msg);
        end

        % Update Visualization
        xy = zeros(numel(threeturtlebot), 2);
        for k = 1:numel(threeturtlebot)
            threeturtlebot(k).plot_handle = turtlebot_plot(threeturtlebot(k));
            [x, y, ~] = turtlebot_get_pose(threeturtlebot(k).pose.msg);
            xy(k,:) = [x, y];
        end
        % Adaptively set axis limits
        xy_min = min(xy, [], 1);
        xy_max = max(xy, [], 1);
        xy_mid = (xy_min + xy_max)/2;
        if (xy_max(1)-xy_min(1) < 6)
            xy_min(1) = xy_mid(1) - 3.0;
            xy_max(1) = xy_mid(1) + 3.0;
        end
        if (xy_max(2)-xy_min(2) < 6)
            xy_min(2) = xy_mid(2) - 3.0;
            xy_max(2) = xy_mid(2) + 3.0;
        end
        
        xy_min = xy_min - 1.0;
        xy_max = xy_max + 1.0;             
        set(axis_handle, 'XLim', [xy_min(1), xy_max(1)], 'YLim', [xy_min(2), xy_max(2)]);

        % Simulator Loop Delay
        pause(1.0/ctrl_rate);        
    end
    
end

%% Turtlebot Pose & Velocity Functions 

function pose = turtlebot_set_pose(x, y, theta)

    pose = ros2message('geometry_msgs/PoseStamped');
    pose.pose.position.x = x;
    pose.pose.position.y = y;

    Qwxyz = angle2quat(theta, 0.0, 0.0, "ZYX");
    pose.pose.orientation.w = Qwxyz(1);
    pose.pose.orientation.x = Qwxyz(2);
    pose.pose.orientation.y = Qwxyz(3);
    pose.pose.orientation.z = Qwxyz(4);

end

function [x, y, theta] = turtlebot_get_pose(pose)
    x = pose.pose.position.x;
    y = pose.pose.position.y;

    Qwxyz = [pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z];
    theta = quat2angle(Qwxyz, "ZYX");

end

function velocity = turtlebot_set_velocity(linear_velocity, angular_velocity)
    velocity = ros2message('geometry_msgs/Twist');
    velocity.linear.x = linear_velocity;
    velocity.angular.z = angular_velocity;
end

function [linear_velocity, angular_velocity] = turtlebot_get_velocity(velocity)
    linear_velocity = velocity.linear.x;
    angular_velocity = velocity.angular.z;
end

function turtlebot_ctrl_callback(msg, turtlebot)
% Callback function for turtlebot control topic
    turtlebot.velocity.msg = msg; 
end

%% Turtlebot Simulator Dynamics & Solver

function dstate = threeturtlebot_dynamics(t, state, ctrl)

    state = reshape(state, [], 3); % [x(m), y(m), theta(rad)]
    ctrl = reshape(ctrl, [], 2); %[linear_velocity (m/s), angular_velocity (m/s)]
    dstate = [ctrl(:,1).*cos(state(:,3)), ctrl(:,1).*sin(state(:,3)), ctrl(:,2)];
    dstate = reshape(dstate, [], 1);

end

function new_turtlebot = turtlebot_state_update(turtlebot, dt)
    
    new_turtlebot = turtlebot;

    numberOfTurtlebots = numel(turtlebot);
    state_mat = zeros(numel(turtlebot), 3);
    ctrl_mat = zeros(numberOfTurtlebots, 2); 
    for k = 1:numel(turtlebot)
        [x, y, theta] = turtlebot_get_pose(turtlebot(k).pose.msg);
        state_mat(k,:) = [x, y, theta];
        [linvel, angvel] = turtlebot_get_velocity(turtlebot(k).velocity.msg);
        ctrl_mat(k,:) = [linvel, angvel];
    end

    state_vec = reshape(state_mat, [numberOfTurtlebots*3, 1]);
    
    [T, S] = ode45(@(t, state) threeturtlebot_dynamics(t, state, ctrl_mat), [0, dt], state_vec);

    new_state_vec = S(end,:);
    new_state_mat = reshape(new_state_vec, [numberOfTurtlebots, 3]);

    for k=1:numberOfTurtlebots
        x = new_state_mat(k,1);
        y = new_state_mat(k,2);
        theta = new_state_mat(k,3);
        new_turtlebot(k).pose.msg = turtlebot_set_pose(x, y, theta);
    end

end

%% Visualization Functions
function plot_handle = turtlebot_plot(turtlebot)

    turtlebotRadius = sqrt(2)*0.306/2;
    numberOfSamples = 60; 

    [x, y, theta] = turtlebot_get_pose(turtlebot.pose.msg);

    directionPolygon = [0.0, -0.2*turtlebotRadius; turtlebotRadius, -0.2*turtlebotRadius; turtlebotRadius, 0.2*turtlebotRadius; 0.0, 0.2*turtlebotRadius] * [cos(theta) -sin(theta); sin(theta) cos(theta)]';
    directionPolygon = [x + directionPolygon(:,1), y + directionPolygon(:,2)];
    bodyPolygon = [x + turtlebotRadius*cos(linspace(0,2*pi,numberOfSamples))',  y + turtlebotRadius*sin(linspace(0,2*pi,numberOfSamples)')];

    if not(isfield(turtlebot, 'plot_handle'))
        plot_handle = zeros(1,2);
        plot_handle(1) = patch('XData', bodyPolygon(:,1), 'YData', bodyPolygon(:,2), 'FaceColor', turtlebot.color, 'EdgeColor', 'none'); 
        plot_handle(2) = patch('XData', directionPolygon(:,1), 'YData', directionPolygon(:,2), 'FaceColor', 0.1*[1 1 1], 'EdgeColor', 'none');
    elseif isempty(turtlebot.plot_handle)
        plot_handle = zeros(1,2);
        plot_handle(1) = patch('XData', bodyPolygon(:,1), 'YData', bodyPolygon(:,2), 'FaceColor', turtlebot.color, 'EdgeColor', 'none'); 
        plot_handle(2) = patch('XData', directionPolygon(:,1), 'YData', directionPolygon(:,2), 'FaceColor', 'k', 'EdgeColor', 'none');
    else
        plot_handle = turtlebot.plot_handle;
        set(plot_handle(1), 'XData', bodyPolygon(:,1), 'YData', bodyPolygon(:,2));
        set(plot_handle(2), 'XData', directionPolygon(:,1), 'YData', directionPolygon(:,2));
    end

end

