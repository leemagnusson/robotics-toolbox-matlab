% make_simscape.m
% Doug Johnston
% Jan, 2017

% Generate a simscape model for the given URDF, hook up input joint angles
% and output joint torque.
%
% This is meant to be run after invoking Haoron's model which generates a
% trajectory
% Required:
% coupling_matrix - how the joints from the URDF relate to the calculated q values
% time_separation - vector of times used for calculations
% q_store - q matrix
% urdf_joint_input - vector of joints names from URDF

arm_number = 1;
filter_time = 'filter_dt';
urdf_dir = [fileparts(mfilename('fullpath')) '/../../../urdfs/'];
model = 'V1_5_1_Arm_URDF_new';
smimport([urdf_dir model '.URDF']); % Note: Matlab 2017a is needed for this call

tmp = [urdf_joint_input{:}];
joints = {tmp.name};
joints = joints(1:15); % only use first 15 joints of model (don't use fixed RCM)
arm_q = [time_log(1:length(q_store{arm_number}))'; coupling_matrix * [q_store{arm_number};zeros(2,length(q_store{arm_number}))]]';
set_robot_q(model, joints, 'arm_q', filter_time);
output_joint_torques(model, joints, 'arm_torque');
set_param(model, 'StopTime', string(time_separation(end)));

% Now you can run the Simscape model. You may wish to adjust the tolerances for the solver

clear tmp
