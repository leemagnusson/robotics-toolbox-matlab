% sets the iterative inverse kinematics parameters
% the current IK parameters are used for inner loop of position control.
% these numbers are provided from the current ROS system.
dt = 0.0025;
num_cycle_translation = 5;
num_cycle_rotation = 2;
v_max = 0.25;
omega_max = 10.5;
eps_translation = 0.001;
eps_rotation = 1 * pi/180;