%% Plot trocar angular velocity
% Plot the angular velocity of the trocar with the tool path recorded from
% stylus sponge. The tool path was saved in 'sponge_tool_path_raw.mat'. The
% joint values were saved in 'stylus_sponge_joint_log_hernia_setup.mat'.
% This script reports the angular velocity of the tool stem in hernia
% setup.
%%

clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('point_boundary_arm_1.0.mat');
load('vertex_patient_body.mat');
load('q_init_setup_hernia.mat');
% load('sponge_tool_path_raw.mat');
% load('suture_tool_path_raw.mat');
load('sponge_tool_path_raw_with_angle.mat');
load('index_joints.mat');
load('coupling_matrix.mat');
% load('stylus_sponge_joint_log_hernia_setup.mat');
load('stylus_sponge_joint_log_hernia_setup_with_angle.mat');
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
index_robot = 0;
for index_bed_adapter = selected_bed_adapter;
    if index_bed_adapter ~=0
        index_robot = index_robot + 1;
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
        transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    end
end

sample_rate = 20;

dt_separation = (time_log(length(q_store{1})) - time_log(1))/(length(q_store{1}) - 1)*sample_rate;
time_separation = 0 : dt_separation : (length(q_store{1}) - 1)/sample_rate * dt_separation;

for i = 1 : 3
    num_sample = 0;
    for j = 1: sample_rate : length(q_store{i})
        num_sample = num_sample + 1;
        q_sd{i}(:,num_sample) = q_store{i}(:,j);
    end
end

% calculate qd
for i = 1 : 3
    for j = 1 : 11
        qd_sd{i}(j,:) = diff(q_sd{i}(j,:))/dt_separation;
    end
end


for i = 1:3
    q = q_sd{i};
    robot_object.transformation_base_ = transformation_base(:,:,i);
    robot_object.CalculateFK(q);
    [jacobian_rcm,jacobian_car,jacobian_all] = robot_object.CalculateJacobianAll;
    for j = 1: length(q)-1
        t_rcm = jacobian_rcm(:,1:4) * qd_sd{i}(6:9,j);
        omega_rcm{i}(:,j) = t_rcm(4:6);
    end
end
% save('../export/omega_rcm.mat','omega_rcm','time_separation');
% save('../export/omega_rcm_with_angle.mat','omega_rcm','time_separation','gripper_left','gripper_right');