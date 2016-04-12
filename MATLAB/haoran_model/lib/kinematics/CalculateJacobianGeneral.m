%% Jacobian on touch point
% function to export Jacobian
% the jacobian here are Jacobian with specific touch point and with
% selected frame number
% [jacobian_touch] = CalculateJacobianGeneral(frames,p_touch,num_frame)
% p_touch: position of touch point in world frame
% num_frame: the index of frame we generate jacobian on.
%%
function [jacobian_touch] = CalculateJacobianGeneral(frames,p_touch,num_frame)
% position of related end effector joints
load('coupling_matrix.mat')
load('index_joints.mat');
z=zeros(3,length(frames));
p=zeros(3,length(frames));
% setup position and z axis of each coordinate frame
for index_arm = 1 : length(frames)
    p(:,index_arm) = frames(1:3,4,index_arm);
    z(:,index_arm) = frames(1:3,3,index_arm);
end
% overall jacobian on touch point with selected frame numbers
index = 1;
for index_arm = 2 : num_frame
    if index_arm ~= index_tool_translate
        jacobian_temp(:,index) = [cross(z(:,index_arm),p_touch - p(:,index_arm));z(:,index_arm)];
        index = index + 1;
    else
        jacobian_temp(:,index) = [z(:,index_arm);zeros(3,1)];
        index = index + 1;
    end
end
% calculate the reduced coupling matrix
if num_frame<=index_pitch_a
    % up to pitch a joint the active joint numbers = total joint numbers
    coupling_matrix_reduced = coupling_matrix(1:num_frame-1,1:num_frame-1);
elseif num_frame==index_pitch_b
    % active joint number is 1 less than total joint number
    coupling_matrix_reduced = coupling_matrix(1:num_frame-1,1:num_frame-2);
else
    % after pitch c joint, active joint number is always 2 less than total
    % joint number
    coupling_matrix_reduced = coupling_matrix(1:num_frame-1,1:num_frame-3);
end

% overall jacobian
jacobian_touch = jacobian_temp * coupling_matrix_reduced;
end
