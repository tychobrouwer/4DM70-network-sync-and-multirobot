positions = logData.positions;  % Size: [T x 3 x 2]
nSteps = size(positions, 1);

% Prepare trajectories for each robot
traj1 = squeeze(positions(:, 1, :)); % [nSteps x 2]
traj2 = squeeze(positions(:, 2, :));
traj3 = squeeze(positions(:, 3, :));

% Extract last density map to show as background
density_map = logData.density_map{end};  % Assume last saved density map
% Define workspace bounds matching your control code
x_range = linspace(-5, 5, size(density_map, 2));
y_range = linspace(-5, 5, size(density_map, 1));

figure;
hold on; grid on; axis equal;
title('TurtleBot Trajectories with Density Map');
xlabel('X [m]');
ylabel('Y [m]');

% Plot density map as background
% imagesc expects X and Y as vectors corresponding to columns and rows
% flip density_map vertically so y=lowest is at bottom
imagesc(x_range, y_range, density_map);
colorbar;
set(gca,'YDir','normal')  % Correct Y axis direction


max_gap = 0.1; % max allowed distance between points to connect, adjust as needed

plot_with_gaps(traj1, max_gap, 'r-', 'Robot 1');
plot_with_gaps(traj2, max_gap, 'm-', 'Robot 2');
plot_with_gaps(traj3, max_gap, 'k-', 'Robot 3');

% Plot start points (excluded from legend)
scatter(traj1(1,1), traj1(1,2), 80, 'ro', 'filled', 'HandleVisibility','off');
scatter(traj2(1,1), traj2(1,2), 80, 'mo', 'filled', 'HandleVisibility','off');
scatter(traj3(1,1), traj3(1,2), 80, 'ko', 'filled', 'HandleVisibility','off');

% Plot end points (excluded from legend)
scatter(traj1(end,1), traj1(end,2), 120, 'r^', 'filled', 'HandleVisibility','off');
scatter(traj2(end,1), traj2(end,2), 120, 'm^', 'filled', 'HandleVisibility','off');
scatter(traj3(end,1), traj3(end,2), 120, 'k^', 'filled', 'HandleVisibility','off');

legend('Location', 'Best');
hold off;
