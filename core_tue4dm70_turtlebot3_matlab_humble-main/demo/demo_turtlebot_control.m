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
%   % Set robot and goal names
%   turtlebotName = 'turtlebot';
%   goalName = 'turtlebotgoal';
%
%   % Start turtlebot control demo
%   turtlebotPoseTopic = sprintf('/mocap/%s/pose', turtlebotName);
%   turtlebotCtrlTopic = sprintf('/%s/cmd_vel_ctrl', turtlebotName);
%   turtlebotGoalPoseTopic = sprintf('/mocap/%s/pose', goalName);
%   demo_turtlebot_control(turtlebotPoseTopic, turtlebotCtrlTopic, turtlebotGoalPoseTopic);

function demo_turtlebot_control(turtlebotPoseTopic, turtlebotCtrlTopic, turtlebotGoalPoseTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
% Created: April 22, 2022
% Modified: May 11, 2023
% Modified: April 08, 2024 (ROS1 to ROS2 Transition)

    %% ROS Node Settings
    % Create the leader-follower node
    nodeName = ['/matlab_turtlebot_control_' int2str(1000000000*rand(1))]; % Node Name
    ctrlNode = ros2node(nodeName);

    % Create the turtlebot control publisher
    [turtlebotCtrlPublisher, turtlebotCtrlMessage] = ros2publisher(ctrlNode, turtlebotCtrlTopic, "geometry_msgs/Twist", 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    
    % Create the turtlebot and goal pose subcribers
    turtlebotPoseSubscriber = ros2subscriber(ctrlNode, turtlebotPoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');
    turtlebotGoalPoseSubscriber = ros2subscriber(ctrlNode, turtlebotGoalPoseTopic, 'geometry_msgs/PoseStamped', 'History', 'keeplast', 'Depth', 1, 'Reliability','reliable');

    %% TurtleBot Control Loop
    disp('Turtlebot Control is running...');
    while (true)
        % Pause for an update rate of at most 10Hz
        pause(0.1);

        % Get the turtlebot and goal information
        turtlebotGoalPose = turtlebotGoalPoseSubscriber.LatestMessage;
        turtlebotPose = turtlebotPoseSubscriber.LatestMessage;
        
        if isempty(turtlebotPose)
            disp('Turtlebot pose is not received!');
            turtlebotPose = ros2message("geometry_msgs/PoseStamped");
        end
        if isempty(turtlebotGoalPose)
            disp('Turtlebot Goal Pose is not receieved!');
            turtlebotGoalPose = turtlebotPose;
        end

        turtlebotPosition = [turtlebotPose.pose.position.x, turtlebotPose.pose.position.y];
        turtlebotOrientation = quat2angle([turtlebotPose.pose.orientation.w,...
                                            turtlebotPose.pose.orientation.x,...
                                            turtlebotPose.pose.orientation.y,...
                                            turtlebotPose.pose.orientation.z]);   
        turtlebotGoalPosition = [turtlebotGoalPose.pose.position.x, turtlebotGoalPose.pose.position.y];
        
        fprintf("Turtlebot Pose: (%.2f, %.2f, %.2f), Goal Position: (%.2f, %.2f)\n",...
            turtlebotPosition(1), turtlebotPosition(2), turtlebotOrientation,...
            turtlebotGoalPosition(1), turtlebotGoalPosition(2));

        % Computer the turtlebot control input                             
        [linvel, angvel] = unicycle_control.unicycle_fwdctrl(turtlebotPosition, turtlebotOrientation, turtlebotGoalPosition, 'Tol', 0.01, 'LinGain', 1, 'AngGain', 1);                                      
        [linvel, angvel] = turtlebot_control.turtlebot_control_governor(linvel, angvel);
        
        % Publish the turtlebot control message
        turtlebotCtrlMessage.linear.x = linvel;
        turtlebotCtrlMessage.angular.z = angvel;
        turtlebotCtrlPublisher.send(turtlebotCtrlMessage);
        
    end
    
end 