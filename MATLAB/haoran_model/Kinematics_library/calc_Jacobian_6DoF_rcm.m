%% Jacobian calculation for 6 DoF rcm point control
% function to export Jacobian that controls the position of the rcm link
% and keep the orientation of the rcm link parallel to the spherical roll
% link. this Jacobian is a special case for calc_Jacobian_general
% by Haoran Yu 3/22/2016
%%
function [J_6DoF_at_rcm] = calc_Jacobian_6DoF_rcm(Frames)
% position of related end effector joints
load('/../data_store/index_joints.mat');

rcm = Frames(1:3,4,index_rcm); % rcm joint
z=zeros(3,length(Frames));
p=zeros(3,length(Frames));
% setup position and z axis of each coordinate frame
for i = 1 : length(Frames)
    p(:,i) = Frames(1:3,4,i);
    z(:,i) = Frames(1:3,3,i);
end
% Calculate the Jacocbian
index = 1;
for i = 2 : index_car
        J_6DoF_at_rcm(:,index) = [cross(z(:,i),rcm - p(:,i));z(:,i)];
        index = index + 1;
end
end
