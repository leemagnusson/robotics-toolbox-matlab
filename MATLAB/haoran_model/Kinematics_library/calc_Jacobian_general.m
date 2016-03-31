%% Jacobian on touch point
% function to export Jacobian
% the jacobian here are Jacobian with specific touch point and with
% selected frame number
% by Haoran Yu 3/16/2016
%%
function [J_touch] = calc_Jacobian_general(Frames,p_touch,frame_num)
% position of related end effector joints
load('/../data_store/coupling_matrix.mat')
load('/../data_store/index_joints.mat');
z=zeros(3,length(Frames));
p=zeros(3,length(Frames));
% setup position and z axis of each coordinate frame
for i = 1 : length(Frames)
    p(:,i) = Frames(1:3,4,i);
    z(:,i) = Frames(1:3,3,i);
end
% overall jacobian on touch point with selected frame numbers
index = 1;
for i = 2 : frame_num
    if i ~= index_tool_translate
        J_temp(:,index) = [cross(z(:,i),p_touch - p(:,i));z(:,i)];
        index = index + 1;
    else
        J_temp(:,index) = [z(:,i);zeros(3,1)];
        index = index + 1;
    end
end
% calculate the reduced coupling matrix
if frame_num<=index_pitch_a
    % up to pitch a joint the active joint numbers = total joint numbers
    A_red = A(1:frame_num-1,1:frame_num-1);
elseif frame_num==index_pitch_b
    % active joint number is 1 less than total joint number
    A_red = A(1:frame_num-1,1:frame_num-2);
else
    % after pitch c joint, active joint number is always 2 less than total
    % joint number
    A_red = A(1:frame_num-1,1:frame_num-3);
end

% overall jacobian
J_touch = J_temp * A_red;
end
