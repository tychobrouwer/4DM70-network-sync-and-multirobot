scenario = 4;


max_gap = 0.1; % max allowed distance between points to connect, adjust as needed

% x_lim = [-3 3];
% y_lim = [-3 3];

% x_lim = [-0.5 3.5];
% y_lim = [-2.5 1.5];

% x_lim = [-0.5 3.5];
% y_lim = [-3 1];

% x_lim = [-2 4];
% y_lim = [-4 2];

% x_lim = [-1 4];
% y_lim = [-3 2];

% x_lim = [0 2.2];
% y_lim = [-1 1.2];

x_lim = [0 4];
y_lim = [-3 1];


positions = logData.positions;  % Size: [T x 3 x 2]
centroids = logData.centroids;  % Size: [T x 3 x 2]
nSteps = size(positions, 1);

% Prepare trajectories for each robot
traj1 = squeeze(positions(:, 1, :)); % [nSteps x 2]
traj2 = squeeze(positions(:, 2, :));
traj3 = squeeze(positions(:, 3, :));

traj1_cent = squeeze(centroids(:, 1, :)); % [nSteps x 2]
traj2_cent = squeeze(centroids(:, 2, :));
traj3_cent = squeeze(centroids(:, 3, :));

% Extract last density map to show as background
density_map = logData.density_map{end};  % Assume last saved density map
% Define workspace bounds matching your control code
x_range = linspace(-5, 5, size(density_map, 2));
y_range = linspace(-5, 5, size(density_map, 1));








f = figure;
tiledlayout(1,3, 'Padding', 'none', 'TileSpacing', 'compact'); 



density_map_1 = logData.density_map{1};

nexttile
hold on; grid on; axis equal;
title('Initial Positions', 'FontSize', 14);
xlabel('X [m]');
ylabel('Y [m]');

% Plot density map as background
imagesc(x_range, y_range, density_map_1);
set(gca,'YDir','normal')  % Correct Y axis direction

% Plot start points (excluded from legend)
scatter(traj1(1,1), traj1(1,2), 80, 'ro', 'filled', 'DisplayName', 'Robot 1');
scatter(traj2(1,1), traj2(1,2), 80, 'mo', 'filled', 'DisplayName', 'Robot 2');
scatter(traj3(1,1), traj3(1,2), 80, 'ko', 'filled', 'DisplayName', 'Robot 3');

scatter(traj1_cent(1,1), traj1_cent(1,2), 120, 'kx', 'HandleVisibility','off');
scatter(traj2_cent(1,1), traj2_cent(1,2), 120, 'kx', 'HandleVisibility','off');
scatter(traj3_cent(1,1), traj3_cent(1,2), 120, 'kx', 'HandleVisibility','off');

xlim(x_lim)
ylim(y_lim)

hold off;

legend('Location', 'southwest', 'FontSize', 14);

ax = gca;
ax.FontSize = 16; 








end_idx = floor(length(logData.density_map)/2);

density_map = logData.density_map{end_idx};

nexttile
hold on; grid on; axis equal;
title('Halfway Positions', 'FontSize', 14);
xlabel('X [m]');

% Plot density map as background
imagesc(x_range, y_range, density_map);
set(gca,'YDir','normal')  % Correct Y axis direction


plot_with_gaps(traj1(1:end_idx,:), max_gap, 'r-', 'Robot 1');
plot_with_gaps(traj2(1:end_idx,:), max_gap, 'm-', 'Robot 2');
plot_with_gaps(traj3(1:end_idx,:), max_gap, 'k-', 'Robot 3');

% Plot start points (excluded from legend)
scatter(traj1(1,1), traj1(1,2), 80, 'ro', 'filled', 'HandleVisibility','off');
scatter(traj2(1,1), traj2(1,2), 80, 'mo', 'filled', 'HandleVisibility','off');
scatter(traj3(1,1), traj3(1,2), 80, 'ko', 'filled', 'HandleVisibility','off');

% Plot end points (excluded from legend)
scatter(traj1(end_idx,1), traj1(end_idx,2), 120, 'r^', 'filled', 'HandleVisibility','off');
scatter(traj2(end_idx,1), traj2(end_idx,2), 120, 'm^', 'filled', 'HandleVisibility','off');
scatter(traj3(end_idx,1), traj3(end_idx,2), 120, 'k^', 'filled', 'HandleVisibility','off');

scatter(traj1_cent(end_idx,1), traj1_cent(end_idx,2), 120, 'kx', 'HandleVisibility','off');
scatter(traj2_cent(end_idx,1), traj2_cent(end_idx,2), 120, 'kx', 'HandleVisibility','off');
scatter(traj3_cent(end_idx,1), traj3_cent(end_idx,2), 120, 'kx', 'HandleVisibility','off');

xlim(x_lim)
ylim(y_lim)

hold off;

ax = gca;
ax.FontSize = 16; 







nexttile
hold on; grid on; axis equal;
title('End Positions', 'FontSize', 14);
xlabel('X [m]');

% Plot density map as background
imagesc(x_range, y_range, density_map);
colorbar;
set(gca,'YDir','normal')  % Correct Y axis direction

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

scatter(traj1_cent(end,1), traj1_cent(end,2), 120, 'kx', 'HandleVisibility','off');
scatter(traj2_cent(end,1), traj2_cent(end,2), 120, 'kx', 'HandleVisibility','off');
scatter(traj3_cent(end,1), traj3_cent(end,2), 120, 'kx', 'HandleVisibility','off');

xlim(x_lim)
ylim(y_lim)

hold off;

ax = gca;
ax.FontSize = 16; 





sgtitle("Multi-Robot Trajectory Scenario " + scenario + " with Centroid Locations", 'FontSize', 18) 
f.Position = [250 850 1000 400];
print("figures_matlab/scenario_" + scenario + "_matlab",'-depsc2');
