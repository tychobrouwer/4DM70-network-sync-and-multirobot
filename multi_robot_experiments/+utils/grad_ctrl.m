%% Collision Avoidance Function
%
% This function computes the collision avoidance control for three robots
% based on their poses and goals. It uses attractive and repulsive forces
% to guide the robots towards their goals while avoiding collisions with
% each other.
%
% @param pose1: The pose of the first robot [x1, y1, theta1].
% @param pose2: The pose of the second robot [x2, y2, theta2].
% @param pose3: The pose of the third robot [x3, y3, theta3].
% @param goal1: The goal position for the first robot [gx1, gy1].
% @param goal2: The goal position for the second robot [gx2, gy2].
% @param goal3: The goal position for the third robot [gx3, gy3].
%
% @return ctrlCmds: The control commands for the three robots
%

function [linvel1, angvel1, linvel2, angvel2, linvel3, angvel3] = grad_ctrl(pose1, pose2, pose3, goal1, goal2, goal3)
    gain1 = 0.5; 
    gain2 = 0.5;
    gain3 = 0.5;

    thresh_scale = 2; %trial and error aanpassen

    wAB = 1;    % B should avoid A
    wAC = 1;    % C should avoid A
    wBC = 0.5;  % C should avoid B

    % Collision-Avoidance Functions & Gradients
    attractive_func = @(x, goal, gain) 0.5*gain*norm(x(:)-goal(:))^2;
    attractive_grad = @(x, goal, gain)  gain*(x(:) - goal(:));
    repulsive_func = @(xA, xB) 0.5*norm(xA(:)-xB(:))^2;
    repulsive_grad = @(xA, xB) (xA(:) - xB(:));

    function grad = compute_grad(xA, xB, xC, goalA, goalB, goalC, gainA, gainB, gainC)
        VA = attractive_func(xA, goalA, gainA);
        VB = attractive_func(xB, goalB, gainB);
        VC = attractive_func(xC, goalC, gainC);
        Vsum = VA + VB + VC;

        VAB = repulsive_func(xA, xB);
        VAC = repulsive_func(xA, xC);
        VBC = repulsive_func(xB, xC);
        W = wAB*VAB + wAC*VAC + wBC*VBC;

        % Gradient of W w.r.t. xA
        grad_VAB = repulsive_grad(xA, xB);
        grad_VAC = repulsive_grad(xA, xC);
        gradW = wAB * grad_VAB + wAC * grad_VAC;

        gradVA = attractive_grad(xA, goalA, gainA);
        % Full gradient
        num = (1 + thresh_scale*W)^2 * gradVA * thresh_scale*W - Vsum * thresh_scale * (1 + thresh_scale*W) * gradW;
        denom = (thresh_scale^2 * W^2);

        grad = -num / denom;  % Negative gradient descent
    end

    % Compute gradients for each robot
    ctrl1 = compute_grad(pose1(1:2), pose2(1:2), pose3(1:2), goal1, goal2, goal3, gain1, gain2, gain3);
    ctrl2 = compute_grad(pose2(1:2), pose1(1:2), pose3(1:2), goal2, goal1, goal3, gain2, gain1, gain3);
    ctrl3 = compute_grad(pose3(1:2), pose1(1:2), pose2(1:2), goal3, goal1, goal2, gain3, gain1, gain2);

    % Compute control commands
    [linvel1, angvel1] = utils.compute_vel_cmd(pose1, ctrl1);
    [linvel2, angvel2] = utils.compute_vel_cmd(pose2, ctrl2);
    [linvel3, angvel3] = utils.compute_vel_cmd(pose3, ctrl3);

    % Clamp the velocities
    [linvel1, angvel1] = utils.control_governor(linvel1, angvel1);
    [linvel2, angvel2] = utils.control_governor(linvel2, angvel2);
    [linvel3, angvel3] = utils.control_governor(linvel3, angvel3);
end
