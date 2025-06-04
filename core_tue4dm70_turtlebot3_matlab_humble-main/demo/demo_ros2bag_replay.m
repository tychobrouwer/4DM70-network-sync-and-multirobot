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
%   % Set ROS topics for replay
%   bagTopics = {};
%
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
%   % Start ROS bag replaying demo
%   demo_ros2bag_replay(bagPath, bagTopics)

function demo_ros2bag_replay(bagPath, bagTopics)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks, d.p.meulendijks@student.tue.nl
% Created: June 04, 2023
% Modified: April 08, 2024 (ROS1 to ROS2 Transition)

    %% Start a ROS Node
    nodeName = ['/matlab_rosbag_replay_' int2str(1000000000*rand(1))];
    replayNode = ros2node(nodeName);

    %% Get the ROS bag
    bag = ros2bagreader(bagPath);
    if not(isempty(bagTopics))
        bag = select(bag, "Topic", bagTopics);
    end

    topicsTable = bag.AvailableTopics;
    publishers = cell(1, height(topicsTable));
    topicsMap = containers.Map;
    for topicCount = 1:height(topicsTable)
        topicName = topicsTable.Row{topicCount};
        topicMessageType = char(topicsTable.MessageType(topicCount));
        topicsMap(topicName) = topicCount;
        publishers{topicCount} = ros2publisher(replayNode, topicName, topicMessageType);
        fprintf("Created a publisher for topic %s.\n", topicName)
    end

    previousTopicTime = bag.StartTime;
    fprintf('Message publishing starts.\n')
    h = waitbar(0,'Publishing messages...');
    for messageCount = 1:bag.NumMessages
        topicName = char(bag.MessageList.Topic(messageCount));
        topicTime = bag.MessageList.Time(messageCount);
        msg = cell2mat(bag.readMessages(messageCount));
        pause(topicTime - previousTopicTime);
        publishers{topicsMap(topicName)}.send(msg);
        previousTopicTime = topicTime;
        waitbar(messageCount/bag.NumMessages, h, sprintf('Publishing messages... %.1f', messageCount/bag.NumMessages*100));
    end
    close(h);
    fprintf("All messages are published!\n");

end