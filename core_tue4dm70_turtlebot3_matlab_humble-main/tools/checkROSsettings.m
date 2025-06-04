function checkROSsettings()

    global ROS_MASTER_URI;
    global ROS_HOSTNAME;
    
    if isempty(ROS_MASTER_URI)
        error('The global variable "ROS_MASTER_URI" is not set!');
    end
    
    if isempty(ROS_HOSTNAME)
        error('The global variable "ROS_HOSTNAME" is not set!');
    end
    
end