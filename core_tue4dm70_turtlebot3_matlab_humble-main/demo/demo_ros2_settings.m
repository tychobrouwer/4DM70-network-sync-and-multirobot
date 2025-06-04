%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% PLEASE MAKE SURE YOUR ROS SETTINGS ARE CORRECT! %%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Example:
%   ROS_DOMAIN_ID = "0";
%   setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID);
%   getenv("ROS_DOMAIN_ID")

function DomainID = demo_ros2_settings(ROS_DOMAIN_ID)
% Author: Omur Arslan, o.arslan@tue.nl
% Created: April 02, 2024
% Modified: April 02, 2024 (ROS1 to ROS2 Transition)
   
    %Set and get the environmental variable for ROS_DOMAIN_ID
    setenv("ROS_DOMAIN_ID", ROS_DOMAIN_ID);
    DomainID = getenv("ROS_DOMAIN_ID");
    
end

