%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE YOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   % Set ROS network settings
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID)
%
%   % Set robot name
%   turtlebotName = 'turtlebot';
%   turtlebotName = 'turtlebot_red';
%   turtlebotName = 'turtlebot_green';
%   turtlebotName = 'turtlebot_blue';
%   turtlebotName = 'turtlebot_magenta';
%   turtlebotName = 'turtlebot_yellow';
%   turtlebotName = 'turtlebot_cyan';
%   turtlebotName = 'turtlebot1';
%   turtlebotName = 'turtlebot2';
%   turtlebotName = 'turtlebot3';
%
%   % Run demo
%   turtlebotPoseTopic = sprintf('/mocap/%s/pose', turtlebotName);
%   demo_turtlebot_print_pose(turtlebotPoseTopic)

function demo_turtlebot_print_pose(turtlebotPoseTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
% Created: May 17, 2023
% Modified: February 02, 2024 (ROS to ROS2 Transition)
% Modified: April 08, 2024 (Renamed)
        
    %% ROS Node Settings
    % Register the leader-follower node at the ROS master
    nodeName = ['/matlab_turtlebot_pose_print' int2str(1000000000*rand(1))]; % Node Name
    poseNode = ros2node(nodeName);

    % Create and register the turtlebot and goal pose subcribers at the ROS master
    turtlebotPoseSubscriber = ros2subscriber(poseNode, turtlebotPoseTopic, 'geometry_msgs/PoseStamped',...
        'History','keeplast','Depth', 1, 'Reliability','besteffort');

    %% TurtleBot Pose Print Loop
    fprintf('Printing TurtleBot Pose [%s]...\n', turtlebotPoseTopic);
    while (true)
        % Pause for an update rate of at most 10Hz
        pause(0.1);

        % Get the latest turtlebot pose information
        turtlebotPose= turtlebotPoseSubscriber.LatestMessage;
        if isempty(turtlebotPose)
            disp('Turtlebot Pose is not received!');
        else
            turtlebotPosition = [turtlebotPose.pose.position.x, turtlebotPose.pose.position.y];
            turtlebotOrientation = quat2angle([turtlebotPose.pose.orientation.w,...
                                               turtlebotPose.pose.orientation.x,...
                                               turtlebotPose.pose.orientation.y,...
                                               turtlebotPose.pose.orientation.z]);   
            fprintf('Current Turtlebot Pose >>> x: %.2f m, y: %.2f m, angle: %4.1f degrees\n', turtlebotPosition(1), turtlebotPosition(2), rad2deg(turtlebotOrientation))
        end

    end

end