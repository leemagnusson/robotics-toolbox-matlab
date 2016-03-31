%% Jacobian calculation for special use
% function to export Jacobian
% the jacobian here are all analytical Jacobians
% by Haoran Yu 3/22/2016
%%
function [J_6DoF_at_rcm] = calc_Jacobian_6DoF_rcm(Arm_Kinematics)
% position of related end effector joints
load('index_joints.mat');
load('coupling_matrix.mat')

eef = Arm_Kinematics(index_eef).Tran_matrix(1:3,4); % end effector at distal wrist joint
rcm = Arm_Kinematics(index_rcm).Tran_matrix(1:3,4); % rcm joint
R_rcm = Arm_Kinematics(index_car).Tran_matrix(1:3,1:3);
% car = Arm_Kinematics(index_car).Tran_matrix(1:3,4); % cartesian arm end joint at spherical roll joint
% wrist = Arm_Kinematics(index_wrist).Tran_matrix(1:3,4); % wrist joint at wrist joint

z=zeros(3,length(Arm_Kinematics));
p=zeros(3,length(Arm_Kinematics));
% setup position and z axis of each coordinate frame
for i = 1 : length(Arm_Kinematics)
    p(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,4);
    z(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,3);
end
% Cartesian arm jacobian for first five joints
index = 1;
for i = 2 : index_car
        J_6DoF_at_rcm(:,index) = [cross(z(:,i),rcm - p(:,i));z(:,i)];
        index = index + 1;
end
end
