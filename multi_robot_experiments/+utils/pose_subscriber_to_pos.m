%% Get the position and orientation from a pose subscriber
%
% This function extracts the position (x, y) and orientation (in radians)
% from a ROS2 pose subscriber message.
%
% Inputs:
%   poseSubscriber: A ROS2 pose subscriber object that contains the
%                   latest pose message.
%
% Outputs:
%   x: The x-coordinate of the pose.
%   y: The y-coordinate of the pose.
%   orientation: The orientation of the pose in radians, converted
%                from quaternion to angle.
%
function [x, y, orientation] = pose_subscriber_to_pos(poseSubscriber)
    turtlebotPose = poseSubscriber.LatestMessage;

    if ~isempty(turtlebotPose)
        x = turtlebotPose.pose.position.x;
        y = turtlebotPose.pose.position.y;
        orientation = quat2angle([turtlebotPose.pose.orientation.w,...
                                  turtlebotPose.pose.orientation.x,...
                                  turtlebotPose.pose.orientation.y,...
                                  turtlebotPose.pose.orientation.z]);
    else 
        x = -100;
        y = -100;
        orientation = -100;
    end
end
