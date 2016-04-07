%% Jacobian calculation for 6 DoF rcm point control
% function to export Jacobian that controls the position of the rcm link
% and keep the orientation of the rcm link parallel to the spherical roll
% link. this Jacobian is a special case for CalculateJacobianGeneral
% [jacobian_6dof_rcm] = CalculateJacobian6DofRCM(frames)
%%
function [jacobian_6dof_rcm] = CalculateJacobian6DofRCM(frames)
% position of related end effector joints
load('index_joints.mat');

rcm = frames(1:3,4,index_rcm); % rcm joint
z_joint=zeros(3,length(frames));
origin_joint=zeros(3,length(frames));
% setup position and z axis of each coordinate frame
for index_arm = 1 : length(frames)
    origin_joint(:,index_arm) = frames(1:3,4,index_arm);
    z_joint(:,index_arm) = frames(1:3,3,index_arm);
end
% Calculate the Jacocbian
index = 1;
for index_arm = 2 : index_car
        jacobian_6dof_rcm(:,index) = [cross(z_joint(:,index_arm),rcm - origin_joint(:,index_arm));z_joint(:,index_arm)];
        index = index + 1;
end
end
