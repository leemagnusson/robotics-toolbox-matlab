%% Jacobian calculation
% function to export Jacobian
% the jacobian here are all analytical Jacobians
% [jacobian_rcm,jacobian_cartesian,jacobian_all] = CalculateJacobianAll(frames)
% jacobian_rcm: 6 by 6 Jacobian that controls only the spherical arm
% jacobian_cartesian: 6 by 5 Jacobian that controls only cartesian arm
% jacobian_all: overall 6 by 11 Jacobian
% by Haoran Yu 3/16/2016
%%
function [jacobian_rcm,jacobian_cartesian,jacobian_all] = CalculateJacobianAll(frames)
% position of related end effector joints
load('index_joints.mat');
load('coupling_matrix.mat');

eef = frames(1:3,4,index_eef); % end effector at distal wrist joint

z=zeros(3,length(frames));
p=zeros(3,length(frames));
% setup position and z axis of each coordinate frame
for i = 1 : length(frames)
    p(:,i) = frames(1:3,4,i);
    z(:,i) = frames(1:3,3,i);
end
% J_temp is the Jacobian with 6 by 13 dimension and takes into account all
% passive/coupled joints
index = 1;
for i = 2 : index_eef
    if i ~= index_tool_translate
        % revolute joint
        jacobian_temp(:,index) = [cross(z(:,i),eef - p(:,i));z(:,i)];
        index = index + 1;
    else
        % prismatic joint
        jacobian_temp(:,index) = [z(:,i);zeros(3,1)];
        index = index + 1;
    end
end

% overall jacobian
jacobian_all = jacobian_temp * coupling_matrix;
jacobian_cartesian = jacobian_all(:,1:5);
jacobian_rcm = jacobian_all(:,6:11);
end
