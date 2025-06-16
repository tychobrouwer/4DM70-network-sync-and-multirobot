function save_log(logData)
    try
        if isempty(logData.timestamp)
            fprintf('Warning: No data to save yet. logData is empty.\n');
            return;
        end
        
        timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        filename = sprintf('data/turtlebot_sim_data_%s.mat', timestamp);
        save(filename, 'logData');
        fprintf('*** Log data saved to: %s with %d samples ***\n', filename, length(logData.timestamp));
        
        % Debug info
        fprintf('Data summary:\n');
        fprintf('  Timestamps: %d entries\n', length(logData.timestamp));
        fprintf('  Positions size: %s\n', mat2str(size(logData.positions)));
        fprintf('  Centroids size: %s\n', mat2str(size(logData.centroids)));
        fprintf('  Density maps: %d entries\n', length(logData.density_map));
        
    catch ME
        fprintf('Error saving data: %s\n', ME.message);
    end
end
