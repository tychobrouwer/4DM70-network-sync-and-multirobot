%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE yOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   % Set ROS network settings
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID)
%
%   % Set leader-follower names
%   leaderName = 'turtlebot_red';
%   follower1Name = 'turtlebot_green';
%   follower2Name = 'turtlebot_blue';
%
%   leaderName = 'turtlebot_magenta';
%   follower1Name = 'turtlebot_yellow';
%   follower2Name = 'turtlebot_cyan';
%
%   leaderName = 'turtlebot1';
%   follower1Name = 'turtlebot2';
%   follower2Name = 'turtlebot3';
%   
%   % Run demo
%   leaderPoseTopic = sprintf('/mocap/%s/pose', leaderName);
%   follower1PoseTopic = sprintf('/mocap/%s/pose', follower1Name);
%   follower1CtrlTopic = sprintf('/%s/cmd_vel_ctrl', follower1Name);
%   follower2PoseTopic = sprintf('/mocap/%s/pose', follower2Name);
%   follower2CtrlTopic = sprintf('/%s/cmd_vel_ctrl', follower2Name);
%   demo_turtlebot_leaderfollowerfollower(leaderPoseTopic, follower1PoseTopic, follower1CtrlTopic, follower2PoseTopic, follower2CtrlTopic);

function demo_turtlebot_leaderfollowerfollower(leaderPoseTopic, follower1PoseTopic, follower1CtrlTopic, follower2PoseTopic, follower2CtrlTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks
% Created: April 21, 2022
% Modified: May 11, 2023 (clean ROS1 version)
% Modified: April 05, 2024 (ROS1 to ROS2 transition)

    %% ROS Node Settings
    % Register the leader-follower node at the ROS master
    nodeName = ['/matlab_turtlebot_leaderfollowerfollower_'  int2str(1000000000*rand(1))]; % Node Name
    leaderfollowerNode = ros2node(nodeName);
    % Create and register the follower control publisher at the ROS master
    [follower1CtrlPublisher, follower1CtrlMessage] = ros2publisher(leaderfollowerNode, follower1CtrlTopic, "geometry_msgs/Twist", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    [follower2CtrlPublisher, follower2CtrlMessage] = ros2publisher(leaderfollowerNode, follower2CtrlTopic, "geometry_msgs/Twist", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');

    % Create and register the leader and follower pose subcribers at the ROS master
    leaderPoseSubscriber = ros2subscriber(leaderfollowerNode, leaderPoseTopic, 'geometry_msgs/PoseStamped',...
        'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
    follower1PoseSubscriber = ros2subscriber(leaderfollowerNode, follower1PoseTopic, 'geometry_msgs/PoseStamped',...
        'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
    follower2PoseSubscriber = ros2subscriber(leaderfollowerNode, follower2PoseTopic, 'geometry_msgs/PoseStamped',...
        'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');

    %% Leader-Follower Control Loop
    disp('Leader-Follower-Follower Control is running...');
    while (true)

        % Pause for an update rate of at most 10Hz
        pause(0.1);

        % Latest poses of the leader and the followers
        leaderPose = leaderPoseSubscriber.LatestMessage;
        follower1Pose = follower1PoseSubscriber.LatestMessage;
        follower2Pose = follower2PoseSubscriber.LatestMessage;

        if isempty(follower1Pose)
            disp('The first follower pose is not receieved!');
            zeroCtrlMessage = ros2message("geometry_msgs/Twist");
            follower1CtrlPublisher.send(zeroCtrlMessage);
            follower2CtrlPublisher.send(zeroCtrlMessage);
            continue;
        end

        if isempty(follower2Pose)
            disp('The second follower pose is not receieved!');
            follower2Pose = follower1Pose;
        end
        
        if isempty(leaderPose)
            disp('The leader pose is not receieved!');
            leaderPose = follower1Pose;
        end

        % Get the leader and follower information
        leaderPosition = [leaderPose.pose.position.x, leaderPose.pose.position.y];
        leaderOrientation = quat2angle([leaderPose.pose.orientation.w,...
                                        leaderPose.pose.orientation.x,...
                                        leaderPose.pose.orientation.y,...
                                        leaderPose.pose.orientation.z]);
        follower1Position = [follower1Pose.pose.position.x, follower1Pose.pose.position.y];
        follower1Orientation = quat2angle([follower1Pose.pose.orientation.w,...
                                           follower1Pose.pose.orientation.x,...
                                           follower1Pose.pose.orientation.y,...
                                           follower1Pose.pose.orientation.z]);   
        follower2Position = [follower2Pose.pose.position.x, follower2Pose.pose.position.y];
        follower2Orientation = quat2angle([follower2Pose.pose.orientation.w,...
                                           follower2Pose.pose.orientation.x,...
                                           follower2Pose.pose.orientation.y,...
                                           follower2Pose.pose.orientation.z]);   
        
        fprintf("Leader: (%.2f, %.2f, %.2f), Follower1: (%.2f, %.2f, %.2f), Follower2: (%.2f, %.2f, %.2f)\n", ...
            leaderPosition(1), leaderPosition(2), leaderOrientation, ...
            follower1Position(1), follower1Position(2), follower1Orientation,...
            follower2Position(1), follower2Position(2), follower2Orientation);

        % Compute control inputs for the follower 1 that follows the leader                             
        [follower1linvel, follower1angvel] = unicycle_control.unicycle_fwdctrl(follower1Position, follower1Orientation, leaderPosition, 'Tol', 0.5, 'LinGain', 1, 'AngGain', 1);                                      
        [follower1linvel, follower1angvel] = turtlebot_control.turtlebot_control_governor(follower1linvel, follower1angvel);
       
        % Compute control inputs for the follower 2 that follows the follower 1                             
        [follower2linvel, follower2angvel] = unicycle_control.unicycle_fwdctrl(follower2Position, follower2Orientation, follower1Position, 'Tol', 0.5, 'LinGain', 1, 'AngGain', 1);                                      
        [follower2linvel, follower2angvel] = turtlebot_control.turtlebot_control_governor(follower2linvel, follower2angvel);
       
        % Publish the follower control message
        follower1CtrlMessage.linear.x = follower1linvel;
        follower1CtrlMessage.angular.z = follower1angvel;
        follower1CtrlPublisher.send(follower1CtrlMessage);
        follower2CtrlMessage.linear.x = follower2linvel;
        follower2CtrlMessage.angular.z = follower2angvel;
        follower2CtrlPublisher.send(follower2CtrlMessage);
        
    end
    
end 