% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE YOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   % Set ROS network settings
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID)
%
%   % Start turtlebot control demo
%   turtlebot_control('turtlebot_red','turtlebot_blue','turtlebot_green')

function turtlebot_control(bot1Name, bot2Name, bot3Name)
    events = [
        2.5774-1,    0;
        1.7331-1,  0.5;
        1.7331-1, -0.5;
    ];

    logData = utils.log_data_handle();
    cleanupObj = onCleanup(@() utils.save_log(logData));

    % Define the event distribution function (density function)
    density = @(xy) sum(exp(-pdist2(xy, events, 'squaredeuclidean')), 2);

    % Voronoi coverage parameters
    workspace_bounds = [-5, 5; -5, 5]; % [xmin, xmax; ymin, ymax]
    grid_resolution = 0.1; % Resolution for numerical integration

    % Initialize ROS 2
    nodeName = ['/matlab_turtlebot_control_' int2str(1000000000*rand(1))]; % Node Name
    ctrlNode = ros2node(nodeName);

    % Create the turtlebot control topics
    bot1PoseTopic = sprintf('/mocap/%s/pose', bot1Name);
    bot1CtrlTopic = sprintf('/%s/cmd_vel_ctrl', bot1Name);
    bot2PoseTopic = sprintf('/mocap/%s/pose', bot2Name);
    bot2CtrlTopic = sprintf('/%s/cmd_vel_ctrl', bot2Name);
    bot3PoseTopic = sprintf('/mocap/%s/pose', bot3Name);
    bot3CtrlTopic = sprintf('/%s/cmd_vel_ctrl', bot3Name);

    % Create the turtlebot control publisher
    [bot1CtrlPublisher, bot1CtrlMessage] = ros2publisher(ctrlNode, bot1CtrlTopic, "geometry_msgs/Twist", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    [bot2CtrlPublisher, bot2CtrlMessage] = ros2publisher(ctrlNode, bot2CtrlTopic, "geometry_msgs/Twist", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    [bot3CtrlPublisher, bot3CtrlMessage] = ros2publisher(ctrlNode, bot3CtrlTopic, "geometry_msgs/Twist", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    
    % Create the turtlebot and goal pose subcribers
    bot1PoseSubscriber = ros2subscriber(ctrlNode, bot1PoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
    bot2PoseSubscriber = ros2subscriber(ctrlNode, bot2PoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
    bot3PoseSubscriber = ros2subscriber(ctrlNode, bot3PoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');

    % Wait for initial poses
    while (true)
        pause(0.1);
        [x,y,orientation] = utils.pose_subscriber_to_pos(bot1PoseSubscriber);
        initPos1 = [x,y,orientation];
        [x,y,orientation] = utils.pose_subscriber_to_pos(bot2PoseSubscriber);
        initPos2 = [x,y,orientation];
        [x,y,orientation] = utils.pose_subscriber_to_pos(bot3PoseSubscriber);
        initPos3 = [x,y,orientation];

        if initPos1(1) ~= -100 && initPos2(1) ~= -100 && initPos3(1) ~= -100
            fprintf("Initial poses received successfully.\n");
            break
        end
    end

    fprintf("Pose 1: (%.2f, %.2f, %.2f) Pose 2: (%.2f, %.2f, %.2f) Pose 3: (%.2f, %.2f, %.2f)\n",...
        initPos1(1), initPos1(2), initPos1(3),...
        initPos2(1), initPos2(2), initPos2(3),...
        initPos3(1), initPos3(2), initPos3(3));

    % TurtleBot Voronoi Coverage Control Loop
    disp('Turtlebot Traditional Voronoi Coverage Control is running...');

    while true
        pause(0.1);

        % Get the current poses of the robots
        [x, y, w] = utils.pose_subscriber_to_pos(bot1PoseSubscriber);
        pose1 = [x, y, w];
        [x, y, w] = utils.pose_subscriber_to_pos(bot2PoseSubscriber);
        pose2 = [x, y, w];
        [x, y, w] = utils.pose_subscriber_to_pos(bot3PoseSubscriber);
        pose3 = [x, y, w];

        if pose1(1) == -100 || pose2(1) == -100 || pose3(1) == -100
            fprintf("Invalid pose received, retrying...\n");
            continue;
        end

        p1 = pose1(1:2); p2 = pose2(1:2); p3 = pose3(1:2);
        robotPoses = [p1; p2; p3];

        % Compute Voronoi cell centroids using Lloyd's algorithm
        [centroids, density_map] = utils.compute_voronoi_centroids(robotPoses, density, workspace_bounds, grid_resolution);

        logData.timestamp(end+1) = now;
        logData.positions(end+1, :, :) = robotPoses;
        logData.centroids(end+1, :, :) = centroids;
        logData.density_map{end+1} = density_map;

        [linvel1, angvel1, linvel2, angvel2, linvel3, angvel3] = utils.grad_ctrl(pose1, pose2, pose3, centroids(1,:), centroids(2,:), centroids(3,:));

        thresh = 5e-2;
        
        % Control Robot 1
        if pdist2(p1, centroids(1,:), 'euclidean') < thresh
            bot1CtrlMessage.linear.x = 0;
            bot1CtrlMessage.angular.z = 0;
            bot1CtrlPublisher.send(bot1CtrlMessage);
        elseif ~isnan(linvel1)
            bot1CtrlMessage.linear.x = linvel1;
            bot1CtrlMessage.angular.z = angvel1;
            bot1CtrlPublisher.send(bot1CtrlMessage);
        end

        % Control Robot 2
        if pdist2(p2, centroids(2,:), 'euclidean') < thresh
            bot2CtrlMessage.linear.x = 0;
            bot2CtrlMessage.angular.z = 0;
            bot2CtrlPublisher.send(bot2CtrlMessage);
        elseif ~isnan(linvel2)
            bot2CtrlMessage.linear.x = linvel2;
            bot2CtrlMessage.angular.z = angvel2;
            bot2CtrlPublisher.send(bot2CtrlMessage);
        end

        % Control Robot 3
        % if pdist2(p3, centroids(3,:), 'euclidean') < thresh
            bot3CtrlMessage.linear.x = 0;
            bot3CtrlMessage.angular.z = 0;
            bot3CtrlPublisher.send(bot3CtrlMessage);
        % elseif ~isnan(linvel3)
        %     bot3CtrlMessage.linear.x = linvel3;
        %     bot3CtrlMessage.angular.z = angvel3;
        %     bot3CtrlPublisher.send(bot3CtrlMessage);
        % end
    end
end

