% sets the iterative inverse kinematics parameters
% the current IK parameters are used for inner loop of position control.
% these numbers are provided from the current ROS system.

% sets the iterative inverse kinematics parameters
dt = 0.0025; % sec
num_cycle_translation = 5; % no unit
num_cycle_rotation = 2; % no unit
v_max = 0.15; % m/s
% v_max = 0.4; % m/s
% omega_max = 10.5; % rad/s
omega_max = 1.5; % rad/s
% eps_translation = 0.001; % m
eps_translation = 0.001; % m
% eps_rotation = 1 * pi/180; % radian
eps_rotation = 1 * pi/180; % radian


dt = 0.0025; % sec
num_cycle_translation = 2; % no unit
num_cycle_rotation = 2; % no unit
v_max = 0.4; % m/s
omega_max = 10.5; % rad/s
eps_translation = 0.002; % m
eps_rotation = 2 * pi/180; % radian