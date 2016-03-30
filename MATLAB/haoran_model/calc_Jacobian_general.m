%% Jacobian calculation
% function to export Jacobian
% the jacobian here are all analytical Jacobians
% by Haoran Yu 3/16/2016
%%
function [J_touch] = calc_Jacobian_general(Arm_Kinematics,p_touch,jnt_frame_num)
% position of related end effector joints
load('coupling_matrix.mat')

z=zeros(3,length(Arm_Kinematics));
p=zeros(3,length(Arm_Kinematics));
% setup position and z axis of each coordinate frame
for i = 1 : length(Arm_Kinematics)
    p(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,4);
    z(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,3);
end
% Cartesian arm jacobian for first five joints
index = 1;
for i = 2 : index_eef
    if i ~= index_tool_translate
        J_temp(:,index) = [cross(z(:,i),eef - p(:,i));z(:,i)];
        index = index + 1;
    else
        J_temp(:,index) = [z(:,i);zeros(3,1)];
        index = index + 1;
    end
end

% overall jacobian
J_all = J_temp * A;
J_touch = J_all(:,1:jnt_frame_num);
end
