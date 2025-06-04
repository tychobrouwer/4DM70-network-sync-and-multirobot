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
%   scanTopic = '/turtlebot/scan';
%   demo_turtlebot_scan(scanTopic);

function demo_turtlebot_scan(scanTopic)
% Authors: Omur Arslan, o.arslan@tue.nl
%          Daan Meulendijks
% Created: March 23, 2022
% Modified: May 11, 2023
    
    %% ROS Node Settings
    nodeName = ['/matlab_turtlebot_scan_' int2str(1000000000*rand(1))]; % Node Name
    % Register node at the ROS master
    scanNode = ros2node(nodeName);
    % Create a subcriber for the scan topic
    scanSubscriber = ros2subscriber(scanNode, scanTopic, 'History', 'keeplast', 'Depth', 1, 'Reliability', 'reliable');
    
    %% Initialize data visualization
    h.figure = figure();
    h.axes = axes('NextPlot', 'add', 'DataAspectRatio', [1 1 1],...
                  'XGrid', 'On','YGrid', 'On', 'Box', 'on',...
                  'XLim', [-4 4], 'YLim', [-4 4]);
    h.plot = plot(h.axes, 0, 0, 'b.');
    
    %% Visualize scan data
    while true % plays indefinitely 'ctrl + c' in command window to stop
        % Terminate if the figure is closed
        if not(ishandle(h.figure))
            break;
        end 
        % Read and visualize scan data
        scan = scanSubscriber.LatestMessage; % Read latest available message sent to the topic
        if not(isempty(scan))
           scan.Ranges(scan.ranges < scan.range_min) = scan.range_max;         
           angles = linspace(scan.angle_min, scan.angle_max,  length(scan.ranges))';
           set(h.plot, 'XData', scan.ranges.*cos(angles), 'YData', scan.ranges.*sin(angles));
        end
        pause(0.05);  % Pause for an update rate of at most 20Hz
    end
end