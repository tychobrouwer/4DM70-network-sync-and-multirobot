%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE YOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID);
%   ros2 topic list
%   topicList = ros2("topic", "list", "DomainID", ROS_DOMAIN_ID);

function topicList = demo_ros2_topic_list(ROS_DOMAIN_ID)
% Author: Omur Arslan, o.arslan@tue.nl
% Created: April 02, 2024
% Modified: April 02, 2024

    %% See all ROS2 topics
    % setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID)
    % ros2 topic list
    topicList = ros2("topic", "list", "DomainID", ROS_DOMAIN_ID);

end