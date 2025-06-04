%% Pose and velocity to linear and angular velocity control
%
% This function computes the linear and angular velocities based on the
% current pose and the desired velocity. It uses a simple proportional
% control approach to adjust the velocities.
%
% @param pose: The current pose of the robot [x, y, theta].
% @param velocity: The desired velocity vector [vx, vy].
%
% @return linvel: The computed linear velocity.
% @return angvel: The computed angular velocity.
%

function [linvel,angvel] = compute_vel_cmd(pose, velocity)
    lingain = 1.0;
    anggain = 2.0;

    velocityTangent = dot(velocity, [cos(pose(3)), sin(pose(3))]);
    velocityNormal = dot(velocity, [-sin(pose(3)), cos(pose(3))]);
    linvel = lingain * velocityTangent;
    angvel = anggain * atan2(velocityNormal,velocityTangent);
end
