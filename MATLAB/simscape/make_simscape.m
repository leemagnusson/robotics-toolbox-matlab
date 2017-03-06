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

% TODO: urdf hopefully not stored in this repo
% model_name = 'V1_Arm_URDF';
% urdf_dir = [fileparts(mfilename('fullpath')) '/../haoran_model/Arm_version_1.0/robots/'];
% urdf_info_mat = [data_dir '/urdf_info_1.0.mat'];

model_name = 'V1.5_Arm_URDF';
urdf_dir = [fileparts(mfilename('fullpath')) '/../haoran_model/Arm_version_1.5_new/robots/'];
urdf_info_mat = [data_dir '/urdf_info_1.5.mat'];

% model_name = 'V2.0_Arm_URDF';
% urdf_dir = [fileparts(mfilename('fullpath')) '/../haoran_model/Arm_version_2.0/robots/'];
% urdf_info_mat = [data_dir '/urdf_info_2.0.mat'];


%% Import
arm_number = 1;
filter_time = 'filter_dt';
model = strrep(model_name,'.','_');

% Note if build server, you should specify stl_relative_path=true in 
% urdf_preprocess.
urdf_preprocess([urdf_dir model_name '.URDF'],'tmp.urdf');
smimport('tmp.urdf','ModelName',model); % Note: Matlab 2017a is needed for this call
delete('tmp.urdf');

%% Instrument
data_dir = [fileparts(mfilename('fullpath')) '/../haoran_model/data'];
load(urdf_info_mat);
tmp = [urdf_joint_input{:}];
joints = {tmp.name};
clear tmp
joints = joints(1:15); % only use first 15 joints of model (don't use fixed RCM)
set_robot_q(model, joints, 'arm_q', filter_time);
output_joint_torques(model, joints, 'arm_torque');


% temporary initial values
filter_dt = 1/400;
arm_q = timeseries( ones(15,2)*diag([0,1]), [0;1]);
set_param(model, 'StopTime', '1');
    
% other useful commands
%arm_q = [time_log(1:length(q_store{arm_number}))'; coupling_matrix * [q_store{arm_number};zeros(2,length(q_store{arm_number}))]]';

% Now you can run the Simscape model. You may wish to adjust the tolerances for the solver


