% ROS2MessageHandle class for creating a handle object with a field msg
%
% Usage:
%   msg_handle = core_ros2tools.ROS2MessageHandle(msg)
% Input:
%   msg: A ROS2 message type
% Output: 
%   msg_handle: ROS2MessageHandle object with property msg_handle.msg
%
% Example:
%   msg = ros2message('geometry_msgs/Twist');
%   msg_handle = core_ros2tools.ROS2MessageHandle(msg)
%   msg_handle.msg

classdef ROS2MessageHandle < handle
% Authors: Omur Arslan, o.arslan@tue.nl
% Created: April 03, 2024
% Modified: April 03, 2024

    properties
        msg
    end
    
    methods
        function obj = ROS2MessageHandle(msg)
            obj.msg = msg;
        end
    end
end