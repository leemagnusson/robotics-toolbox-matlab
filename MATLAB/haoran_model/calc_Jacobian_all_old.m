%% Jacobian calculation
% function to export Jacobian
% the jacobian here are all analytical Jacobians
% by Haoran Yu 3/16/2016
%%
function [J_rcm,J_car,J_car_6DoF,J_all] = calc_Jacobian_all_old(Arm_Kinematics)
% position of related end effector joints
load('index_joints.mat');

eef = Arm_Kinematics(index_eef).Tran_matrix(1:3,4); % end effector at distal wrist joint
rcm = Arm_Kinematics(index_rcm).Tran_matrix(1:3,4); % rcm joint
car = Arm_Kinematics(index_car).Tran_matrix(1:3,4); % cartesian arm end joint at spherical roll joint
wrist = Arm_Kinematics(index_wrist).Tran_matrix(1:3,4); % wrist joint at wrist joint

z=zeros(3,length(Arm_Kinematics));
p=zeros(3,length(Arm_Kinematics));
% setup position and z axis of each coordinate frame
for i = 1 : length(Arm_Kinematics)
    p(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,4);
    z(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,3);
end
% Cartesian arm jacobian for first five joints
index = 1;
for i = 2 : index_car-1
    J_car(:,index) = [cross(z(:,i),car - p(:,i));z(:,i)];
    index = index + 1;
end
% Cartesian arm jacobian for first five joints and roll joint of spherical arm
index = 1;
for i = 2 : index_car
    J_car_6DoF(:,index) = [cross(z(:,i),car - p(:,i));z(:,i)];
    index = index + 1;
end
% rcm arm 6DoF jacobian
r = wrist - rcm;
r_cross = CPM(-r);
B = r_cross * [z(:,index_car) z(:,index_pitch_c) z(:,index_tool_rotate)];
J_rcm = [B(:,1:2)                              z(:,index_tool_translate)     B(:,3)                       cross(z(:,index_wrist),eef - p(:,index_wrist))    cross(z(:,index_eef),eef - p(:,index_eef));...
    z(:,index_car)     z(:,index_pitch_c)      zeros(3,1)                    z(:,index_tool_rotate)       z(:,index_wrist)                                  z(:,index_eef)];

% overall jacobian
J_all = [J_car J_rcm];

end
