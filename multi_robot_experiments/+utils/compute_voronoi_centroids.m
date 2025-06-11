%% Compute Voronoi cell centroids
%
% Inputs:
%   robotPoses: Robot positions [N x 2]
%   density_func: Function handle for density distribution
%   workspace_bounds: Workspace boundaries [2 x 2] = [xmin,xmax; ymin,ymax]
%   grid_resolution: Grid resolution for numerical integration
%
% Outputs:
%   centroids: Centroid of each robot's Voronoi cell [N x 2]
%   density_map: Map of Voronoi densities
%
function [centroids, density_map] = compute_voronoi_centroids(robotPoses, density_func, workspace_bounds, grid_resolution)
    num_robots = size(robotPoses, 1);
    
    % Create grid for numerical integration
    x_range = workspace_bounds(1,1):grid_resolution:workspace_bounds(1,2);
    y_range = workspace_bounds(2,1):grid_resolution:workspace_bounds(2,2);
    [X, Y] = meshgrid(x_range, y_range);
    grid_points = [X(:), Y(:)];
    
    % Compute Voronoi partition
    % For each grid point, find the closest robot
    distances = pdist2(grid_points, robotPoses, 'euclidean');
    [~, voronoi_assignment] = min(distances, [], 2);
    
    % Compute density at each grid point
    density_values =  density_func(grid_points);
    density_map = reshape(density_values, size(X));

    % Compute centroid for each robot's Voronoi cell
    centroids = zeros(num_robots, 2);
    
    for robot_idx = 1:num_robots
        % Find grid points in this robot's Voronoi cell
        cell_points = grid_points(voronoi_assignment == robot_idx, :);
        cell_densities = density_values(voronoi_assignment == robot_idx);
        
        if isempty(cell_points)
            % If no points assigned, use current robot position
            centroids(robot_idx, :) = robotPoses(robot_idx, :);
            continue;
        end
        
        % Compute mass and first moment for centroid calculation
        total_mass = sum(cell_densities) * grid_resolution^2;
        
        if total_mass > 0
            % Weighted centroid: ∫∫ p(x,y) * [x,y] dx dy / ∫∫ p(x,y) dx dy
            weighted_x = sum(cell_points(:,1) .* cell_densities) * grid_resolution^2;
            weighted_y = sum(cell_points(:,2) .* cell_densities) * grid_resolution^2;
            
            centroids(robot_idx, 1) = weighted_x / total_mass;
            centroids(robot_idx, 2) = weighted_y / total_mass;
        else
            % If no density in cell, use geometric centroid
            centroids(robot_idx, :) = mean(cell_points, 1);
        end
        
        % Ensure centroid is within workspace bounds
        centroids(robot_idx, 1) = max(workspace_bounds(1,1), min(workspace_bounds(1,2), centroids(robot_idx, 1)));
        centroids(robot_idx, 2) = max(workspace_bounds(2,1), min(workspace_bounds(2,2), centroids(robot_idx, 2)));
    end
end
