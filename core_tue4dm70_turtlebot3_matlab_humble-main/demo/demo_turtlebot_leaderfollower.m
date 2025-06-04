%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE yOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   % Set ROS network settings
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID);
%
%   % Set leader-follower names
%   leaderName = 'turtlebot_red';
%   followerName = 'turtlebot_green';
%   leaderName = 'turtlebot_magenta';
%   followerName = 'turtlebot_yellow';
%   leaderName = 'turtlebot1';
%   followerName = 'turtlebot2';
%
%   % Run demo
%   leaderPoseTopic = sprintf('/mocap/%s/pose', leaderName);
%   followerPoseTopic = sprintf('/mocap/%s/pose', followerName);
%   followerCtrlTopic = sprintf('/%s/cmd_vel_ctrl', followerName);
%   demo_turtlebot_leaderfollower(leaderPoseTopic, followerPoseTopic, followerCtrlTopic);

function demo_turtlebot_leaderfollower(leaderPoseTopic, followerPoseTopic, followerCtrlTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks
% Created: March 23, 2022
% Modified: May 11, 2023
% Modified: April 08, 2024 (ROS1 to ROS2 Transition)

    %% ROS Node Settings
    % Register the leader-follower node at the ROS master
    nodeName = ['/matlab_turtlebot_leaderfollower_' int2str(1000000000*rand(1))]; % Node Name
    leaderfollowerNode = ros2node(nodeName);
    % Create and register the follower control publisher at the ROS master
    [followerCtrlPublisher, followerCtrlMessage] = ros2publisher(leaderfollowerNode, followerCtrlTopic, "geometry_msgs/Twist",...
        'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    
    % Create and register the leader and follower pose subcribers at the ROS master
    leaderPoseSubscriber = ros2subscriber(leaderfollowerNode, leaderPoseTopic, 'geometry_msgs/PoseStamped',...
        'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
    followerPoseSubscriber = ros2subscriber(leaderfollowerNode, followerPoseTopic, 'geometry_msgs/PoseStamped',...
        'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
    
    %% Leader-Follower Control Loop
    disp('Leader-Follower Control is running...');
    while (true)
        % Pause for an update rate of at most 10Hz
        pause(0.1);
        % Get the lastest information from the ROS network
        leaderPose = leaderPoseSubscriber.LatestMessage;
        followerPose = followerPoseSubscriber.LatestMessage;
        if isempty(followerPose)
            disp('Follower pose is not received!');
            followerPose = ros2message('geometry_msgs/PoseStamped');
        end
        if isempty(leaderPose)
            disp('Leader pose is not received!');
            leaderPose = followerPose;
        end

        % Get the leader and follower information
        leaderPosition = [leaderPose.pose.position.x, leaderPose.pose.position.y];
        leaderOrientation = quat2angle([leaderPose.pose.orientation.w,...
                                        leaderPose.pose.orientation.x,...
                                        leaderPose.pose.orientation.y,...
                                        leaderPose.pose.orientation.z]); 
        followerPosition = [followerPose.pose.position.x, followerPose.pose.position.y];
        followerOrientation = quat2angle([followerPose.pose.orientation.w,...
                                         followerPose.pose.orientation.x,...
                                         followerPose.pose.orientation.y,...
                                         followerPose.pose.orientation.z]);      
        fprintf("Leader Pose: (%.2f, %.2f, %.2f), Follower Pose(%.2f, %.2f, %.2f)\n", ...
            leaderPosition(1), leaderPosition(2), leaderOrientation, ...
            followerPosition(1), followerPosition(2), followerOrientation);

        % Computer the follower control input                             
        [linvel, angvel] = unicycle_control.unicycle_fwdctrl(followerPosition, followerOrientation, leaderPosition, ...
                    'Tol', 0.5, 'LinGain', 1, 'AngGain', 1);                                      
        [linvel, angvel] = turtlebot_control.turtlebot_control_governor(linvel, angvel);
        
        % Publish the follower control message
        followerCtrlMessage.linear.x = linvel;
        followerCtrlMessage.angular.z = angvel;
        followerCtrlPublisher.send(followerCtrlMessage);
    end
end