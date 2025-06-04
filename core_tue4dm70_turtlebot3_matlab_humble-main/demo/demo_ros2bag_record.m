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
%   % Set ROS topics for recording
%   bagTopics = {"/mocap/turtlebot/pose", "/turtlebot/cmd_vel"};
%
%   bagTopics = {"/mocap/turtlebot_red/pose", "/mocap/turtlebot_green/pose", "/mocap/turtlebot_blue/pose", "/turtlebot_red/cmd_vel", "/turtlebot_green/cmd_vel", "/turtlebot_blue/cmd_vel"};
%
%   bagTopics = {"/mocap/turtlebot_magenta/pose", "/mocap/turtlebot_yellow/pose", "/mocap/turtlebot_cyan/pose", "/turtlebot_magenta/cmd_vel", "/turtlebot_yellow/cmd_vel", "/turtlebot_cyan/cmd_vel"};
%
%   bagTopics = {"/mocap/turtlebot1/pose", "/mocap/turtlebot2/pose", "/mocap/turtlebot3/pose", "/turtlebot1/cmd_vel", "/turtlebot2/cmd_vel", "/turtlebot3/cmd_vel"};
%
%   % Set path for bag file
%   bagPath = fullfile(getenv("HOME"), "Documents", "tmp", "demo_ros2bag");
%
%   % Start ROS bag recording demo
%   demo_ros2bag_record(bagPath, bagTopics)

function demo_ros2bag_record(bagPath, bagTopics)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks, d.p.meulendijks@student.tue.nl
% Created: June 04, 2023
% Modified: April 08, 2024 (ROS1 to ROS2 Transition)

    
    %% Start a ROS Node
    nodeName = ['/matlab_bagwriter_' int2str(1000000000*rand(1))];
    recordNode = ros2node(nodeName);

    %% Get the ROS bag
    if exist(bagPath,'dir')
        rmdir(bagPath, 's')
    end
    bagwriter = ros2bagwriter(bagPath);
    cleanup = onCleanup(@() demo_rosbag_record_cleanup(bagwriter));

    %% Create publishers for all topics in the bag file
    subscribers = cell(length(bagTopics), 1);
    for topicCount = 1:length(bagTopics)
        topicName = bagTopics{topicCount};
        numberofattempts = 0;
        maximumattempts = 10;
        while numberofattempts < maximumattempts
            numberofattempts = numberofattempts + 1;
            try
                subscribers{topicCount} = ros2subscriber(recordNode, topicName, @(data) callback_msg(data, recordNode, bagwriter, topicName), 'History', 'keeplast', 'Depth', 1, 'Reliability','besteffort');
                fprintf("A subscriber is created for topic %s.\n", topicName);
                numberofattempts = maximumattempts;
            catch ME
                % Display error message and retry
                disp(['Error creating subscriber: ', ME.message]);
                fprintf('Attempt %d/%d. Retrying again...\n', numberofattempts, maximumattempts);
                pause(1); % Wait for a short time before retrying
            end
        end
    end

    %% Write data to bag
    disp("Started recording to " + bagPath + ". Press ctrl + c to stop recording.")
    numberOfDots = 0;
    while true % plays indefinitely 'ctrl + c' in command window to stop
        if (numberOfDots >= 50)
            numberOfDots = 0;
            fprintf('\n');
        end
        fprintf('.');
        numberOfDots = numberOfDots + 1;
        pause(0.1);
    end
end


function callback_msg(data, node, writer, topicName)
    write(writer, topicName, ros2time(node, "now"), data)
end


function demo_rosbag_record_cleanup(bagwriter)
    bagpath = bagwriter.Path;
    fprintf('\nClose the bag file, remove the ros2bagwriter object from memory, and clear the associated object.\n');
    delete(bagwriter)
    clear bagwriter
    fprintf('Bag file is saved to the folder %s.\n', bagpath);
end


