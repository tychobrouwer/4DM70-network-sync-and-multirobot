%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE yOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID)
%   goalPoseTopic = '/mocap/turtlebotgoal/pose';
%   leaderPoseTopic = '/mocap/turtlebot1/pose';
%   leaderCtrlTopic = '/turtlebot1/cmd_vel_ctrl';
%   follower1PoseTopic = '/mocap/turtlebot2/pose';
%   follower1CtrlTopic = '/turtlebot2/cmd_vel_ctrl';
%   follower2PoseTopic = '/mocap/turtlebot3/pose';
%   follower2CtrlTopic = '/turtlebot3/cmd_vel_ctrl';
%   demo_turtlebot_goalleaderfollowerfollower(goalPoseTopic, leaderPoseTopic, leaderCtrlTopic, follower1PoseTopic, follower1CtrlTopic, follower2PoseTopic, follower2CtrlTopic);

function demo_turtlebot_goalleaderfollowerfollower(goalPoseTopic, leaderPoseTopic, leaderCtrlTopic, follower1PoseTopic, follower1CtrlTopic, follower2PoseTopic, follower2CtrlTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks
% Created: April 21, 2022
% Modified: May 11, 2023
% Modified: April 08, 2024 (ROS1 to ROS2 Transition)
    
    %% ROS Node Settings
    % Create the control node
    nodeName = ['/matlab_turtlebot_goalleaderfollowerfollower_'  int2str(1000000000*rand(1))]; % Node Name
    ctrlNode = ros2node(nodeName);
    % Create the leader and follower control publisher
    [leaderCtrlPublisher, leaderCtrlMessage] = ros2publisher(ctrlNode, leaderCtrlTopic, 'geometry_msgs/Twist', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    [follower1CtrlPublisher, follower1CtrlMessage] = ros2publisher(ctrlNode, follower1CtrlTopic, 'geometry_msgs/Twist', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    [follower2CtrlPublisher, follower2CtrlMessage] = ros2publisher(ctrlNode, follower2CtrlTopic, 'geometry_msgs/Twist', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');

    % Create the leader and follower pose subcribers
    goalPoseSubscriber = ros2subscriber(ctrlNode, goalPoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    leaderPoseSubscriber = ros2subscriber(ctrlNode, leaderPoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    follower1PoseSubscriber = ros2subscriber(ctrlNode, follower1PoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    follower2PoseSubscriber = ros2subscriber(ctrlNode, follower2PoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');

    %% Leader-Follower Control Loop
    disp('Leader-Follow Control is running...');
    while (true)

        % Pause for an update rate of at most 10Hz
        pause(0.1);

        % Get the latest poses of the goal, leader and followers
        goalPose = goalPoseSubscriber.LatestMessage;
        leaderPose = leaderPoseSubscriber.LatestMessage;
        follower1Pose = follower1PoseSubscriber.LatestMessage;
        follower2Pose = follower2PoseSubscriber.LatestMessage;
        if isempty(goalPose)
            disp('The goal pose is not receieved!');
            zeroCtrlMessage = ros2message("geometry_msgs/Twist");
            leaderCtrlPublisher.send(zeroCtrlMessage);
            follower1CtrlPublisher.send(zeroCtrlMessage);
            follower2CtrlPublisher.send(zeroCtrlMessage);
            continue;
        end
        if isempty(leaderPose)
            disp('The leader pose is not receieved!');
            zeroCtrlMessage = ros2message("geometry_msgs/Twist");
            leaderCtrlPublisher.send(zeroCtrlMessage);
            follower1CtrlPublisher.send(zeroCtrlMessage);
            follower2CtrlPublisher.send(zeroCtrlMessage);
            continue;
        end
        if isempty(follower1Pose)
            disp('The follower1 pose is not receieved!');
            zeroCtrlMessage = ros2message("geometry_msgs/Twist");
            leaderCtrlPublisher.send(zeroCtrlMessage);
            follower1CtrlPublisher.send(zeroCtrlMessage);
            follower2CtrlPublisher.send(zeroCtrlMessage);
            continue;
        end
        if isempty(follower2Pose)
            disp('The follower2 pose is not receieved!');
            zeroCtrlMessage = ros2message("geometry_msgs/Twist");
            leaderCtrlPublisher.send(zeroCtrlMessage);
            follower1CtrlPublisher.send(zeroCtrlMessage);
            follower2CtrlPublisher.send(zeroCtrlMessage);
            continue;
        end

        % Get the leader, follower, and goal information
        goalPosition = [goalPose.pose.position.x, goalPose.pose.position.y];
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
        
        fprintf("Goal: (%.2f, %.2f), Lead: (%.2f, %.2f, %.2f), Follow1: (%.2f, %.2f, %.2f), Follow2: (%.2f, %.2f, %.2f)\n", ...
            goalPosition(1), goalPosition(2), ...
            leaderPosition(1), leaderPosition(2), leaderOrientation, ...
            follower1Position(1), follower1Position(2), follower1Orientation,...
            follower2Position(1), follower2Position(2), follower2Orientation);

        % Compute control inputs for the leader towards the goal                               
        [leaderlinvel, leaderangvel] = unicycle_control.unicycle_fwdctrl(leaderPosition, leaderOrientation, goalPosition, 'Tol', 0.01, 'LinGain', 1, 'AngGain', 1);
        [leaderlinvel, leaderangvel] = turtlebot_control.turtlebot_control_governor(leaderlinvel, leaderangvel);
        % Compute control inputs for the follower 1 that follows the leader  
        [follower1linvel, follower1angvel] = unicycle_control.unicycle_fwdctrl(follower1Position, follower1Orientation, leaderPosition, 'Tol', 0.5, 'LinGain', 1, 'AngGain', 1);                                      
        [follower1linvel, follower1angvel] = turtlebot_control.turtlebot_control_governor(follower1linvel, follower1angvel);
        % Compute control inputs for the follower 2 that follows the follower 1                             
        [follower2linvel, follower2angvel] = unicycle_control.unicycle_fwdctrl(follower2Position, follower2Orientation, follower1Position, 'Tol', 0.5, 'LinGain', 1, 'AngGain', 1);                                      
        [follower2linvel, follower2angvel] = turtlebot_control.turtlebot_control_governor(follower2linvel, follower2angvel);
       
        % Publish the leader & follower control message
        leaderCtrlMessage.linear.x = leaderlinvel;
        leaderCtrlMessage.angular.z = leaderangvel;
        leaderCtrlPublisher.send(leaderCtrlMessage);        
        follower1CtrlMessage.linear.x = follower1linvel;
        follower1CtrlMessage.angular.z = follower1angvel;
        follower1CtrlPublisher.send(follower1CtrlMessage);
        follower2CtrlMessage.linear.x = follower2linvel;
        follower2CtrlMessage.angular.z = follower2angvel;
        follower2CtrlPublisher.send(follower2CtrlMessage);
        
    end
    
end 