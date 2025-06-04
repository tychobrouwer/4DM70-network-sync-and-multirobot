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
set(p, 'FaceColor', 'red', 'EdgeColor', 'none');

% Add lighting and labels
camlight; lighting phong
xlabel('y');
ylabel('z_1');
zlabel('z_2');
grid on;
axis equal;
view(3);
xlim([-5, 5])
ylim([-10, 5])
zlim([-8, 12])
