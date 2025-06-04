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
%   ctrlTopic = sprintf('/%s/cmd_vel_ctrl', turtlebotName);
%   demo_turtlebot_teleop(ctrlTopic);

function demo_turtlebot_teleop(ctrlTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks
% Created: March 23, 2022
% Modified: May 11, 2023
% Modified: April 08, 2024 (ROS1 to ROS2 adaptation)

    %% ROS Node Settings
    % Register a turtlebot teleoperation node at the ROS master
    nodeName = ['/matlab_turtlebot_teleop_' int2str(1000000000*rand(1))];
    teleopNode = ros2node(nodeName);
    % Create and register a turtlebot control publisher at the ROS master
    ctrl_pub = ros2publisher(teleopNode, ctrlTopic, "geometry_msgs/Twist");
    ctrl_msg = ros2message("geometry_msgs/Twist");
    ctrl_msg_handle = core_ros2tools.ROS2MessageHandle(ctrl_msg);
    
    %% Visualization for turtlebot control interface
    h.figure = figure('KeyPressFcn', @(src, data) KeyPressFcn(src, data, ctrl_msg_handle),...
                      'DeleteFcn', @(src, event) DeleteFcn (src, event, ctrl_msg_handle));
    h.axes = axes('Position', [0.0, 0.0, 1.0, 1.0], 'XLim', [0, 1], 'YLim', [0,1], 'Visible', 'off');
    inputText = [sprintf('Motion Control Keys:\n')...
                 sprintf('      w                  w/x : increase/decrease linear velocity\n')...
                 sprintf(' a   s   d              a/d : increase/decrease angular velocity\n')...
                 sprintf('      x                   s : stop')];
    h.keytext = text(h.axes, 0.1, 0.7, inputText, 'FontSize', 12, 'BackgroundColor', 'w');
    fcmdtext = @(cmd) sprintf('>> Current Linear Velocity: %.2f \n>> Current Angular Velocity: %.2f',...
                              cmd.linear.x, cmd.angular.z);
    h.cmdtext = text(h.axes, 0.1, 0.3, fcmdtext(ctrl_msg_handle.msg), 'FontSize', 12, 'BackgroundColor', 'w');
    
    cleanupObj = onCleanup(@() demo_turtlebot_teleop_cleanup(h.figure));

    %% Start teleoperation loop
    while (true)
        % Terminate if the figure is closed
        if not(ishandle(h.figure))
            break;
        end 
        % Send turtlebot control message
        ctrl_pub.send(ctrl_msg_handle.msg);
        % Update turtlebot control info
        set(h.cmdtext, 'String',  fcmdtext(ctrl_msg_handle.msg));
        % Pause for an update rate of at most 10Hz
        pause(0.1); 
    end
end

function KeyPressFcn(src, data, ctrl_msg_handle)
    % Callback function for the figure-keypressed event to set up turtlebot control inputs 
    
    % Turtlebot control limits
    maxLinSpeed = 0.26;
    maxAngSpeed = 1.82;

    % Adjust turtlebot control inputs
    switch data.Key
        case 'w'
            ctrl_msg_handle.msg.linear.x = ctrl_msg_handle.msg.linear.x + 0.1*maxLinSpeed;
            ctrl_msg_handle.msg.linear.x = min(ctrl_msg_handle.msg.linear.x, maxLinSpeed);
        case 'x'
            ctrl_msg_handle.msg.linear.x = ctrl_msg_handle.msg.linear.x - 0.1*maxLinSpeed;
            ctrl_msg_handle.msg.linear.x = max(ctrl_msg_handle.msg.linear.x, -maxLinSpeed);
        case 'a'
            ctrl_msg_handle.msg.angular.z = ctrl_msg_handle.msg.angular.z + 0.1 * maxAngSpeed;
            ctrl_msg_handle.msg.angular.z = min(ctrl_msg_handle.msg.angular.z, maxAngSpeed);
        case 'd'
            ctrl_msg_handle.msg.angular.z = ctrl_msg_handle.msg.angular.z - 0.1 * maxAngSpeed;
            ctrl_msg_handle.msg.angular.z = max(ctrl_msg_handle.msg.angular.z, -maxAngSpeed);
        case 's'
            ctrl_msg_handle.msg.linear.x = 0;
            ctrl_msg_handle.msg.angular.z = 0;
    end
end

function DeleteFcn(src, event, ctrl_msg_handle)
    % Callback function for the figure-closed event  
    % Stop turtlebot by setting control inputs to zero
    ctrl_msg_handle.msg.linear.x = 0;
    ctrl_msg_handle.msg.angular.z = 0;
end

function demo_turtlebot_teleop_cleanup(figure_handle)
    %Cleanup tasks upon function completion
    delete(figure_handle);
end