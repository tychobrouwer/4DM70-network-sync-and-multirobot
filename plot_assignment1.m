% Define grid limits and resolution
y = linspace(-20, 20, 300);
z1 = linspace(-20, 20, 300);
z2 = linspace(-20, 20, 300);

% Create 3D grid
[Y, Z1, Z2] = meshgrid(y, z1, z2);

% Calculate H at each grid point
H = (1/4)*(Y.^4 + Z1.^2) - 3*Y.^2 + + 4.7*Y + (1/4)*(Z2 - 2.236).^2 - 1.2499;

% Plot the implicit surface H = 0 using isosurface
figure;
p = patch(isosurface(Y, Z1, Z2, H, 0));
isonormals(Y, Z1, Z2, H, p)
set(p, 'FaceColor', 'blue', 'EdgeColor', 'none');

ax = gca;
ax.FontSize = 24;


% Add lighting and labels
camlight("right"); lighting gouraud
xlabel('y', 'FontSize', 32);
ylabel('z_1', 'FontSize', 32);
zlabel('z_2', 'FontSize', 32);
grid on;
view(3);
xlim([-5, 1])
ylim([-10, 10])
zlim([-10+2.236, 10+2.236])




clear all;

y = linspace(-20, 20, 300);
z2 = linspace(-20, 20, 300);

figure;
z1_val = -2.236;

% Create 3D grid
[Y, Z2] = meshgrid(y, z2);

% Calculate H at each grid point
H = (1/4)*(Y.^4 + z1_val.^2) - 3*Y.^2 + + 4.7*Y + (1/4)*(Z2 - 2.236).^2 - 1.2499;

contour(Y, Z2, H', [0, 0], 'b', 'LineWidth', 2)
ylabel('y', 'FontSize', 32);
xlabel('z_1', 'FontSize', 32);
title('', 'FontSize', 32)
grid on;
xlim([-10+2.236, 10+2.236])
ylim([-5, 1])

ax = gca;
ax.FontSize = 24;




clear all;

z1 = linspace(-20, 20, 300);
z2 = linspace(-20, 20, 300);

figure;
y_val = -2.7739;

% Create 3D grid
[Z1, Z2] = meshgrid(z1, z2);

% Calculate H at each grid point
H = (1/4)*(y_val.^4 + Z1.^2) - 3*y_val.^2 + + 4.7*y_val + (1/4)*(Z2 - 2.236).^2 - 1.2499;

contour(Z1, Z2, H', [0, 0], 'b', 'LineWidth', 2)
xlabel('z_1', 'FontSize', 32);
ylabel('z_2', 'FontSize', 32);
grid on;
xlim([-10+2.236, 10+2.236])
ylim([-10, 10])

ax = gca;
ax.FontSize = 24;
