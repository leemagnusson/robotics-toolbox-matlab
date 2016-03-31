%% Jacobian calculation
% function to export Jacobian
% the jacobian here are all analytical Jacobians
% J_rcm: 6 by 6 Jacobian that controls only the spherical arm
% J_car: 6 by 5 Jacobian that controls only cartesian arm
% J_all: overall 6 by 11 Jacobian
% by Haoran Yu 3/16/2016
%%
function [J_rcm,J_car,J_all] = calc_Jacobian_all(Frames)
% position of related end effector joints
load('/../data_store/index_joints.mat');
load('/../data_store/coupling_matrix.mat')

eef = Frames(1:3,4,index_eef); % end effector at distal wrist joint

z=zeros(3,length(Frames));
p=zeros(3,length(Frames));
% setup position and z axis of each coordinate frame
for i = 1 : length(Frames)
    p(:,i) = Frames(1:3,4,i);
    z(:,i) = Frames(1:3,3,i);
end
% J_temp is the Jacobian with 6 by 13 dimension and takes into account all
% passive/coupled joints
index = 1;
for i = 2 : index_eef
    if i ~= index_tool_translate
        % revolute joint
        J_temp(:,index) = [cross(z(:,i),eef - p(:,i));z(:,i)];
        index = index + 1;
    else
        % prismatic joint
        J_temp(:,index) = [z(:,i);zeros(3,1)];
        index = index + 1;
    end
end

% overall jacobian
J_all = J_temp * A;
J_car = J_all(:,1:5);
J_rcm = J_all(:,6:11);
end
