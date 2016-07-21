%% Resolved rates single arm
% This code runs resolved rates for single arm with desired task space
% input. The IK is for 6DOF spherical arm control. It cooresponds with the
% IK mode 'Spherical 6'. The end-effector is at the distal wrist joint
% origin. The IK parameters could be set from InitIKParameters.m.
%%
% init
clc
clear all
close all
load('vertex_arm_origin_1.0.mat')
load('arm_version_1.0.mat')
load('coupling_matrix.mat')
% init resolved rates
InitIKParameters;
q_init = [0,0,0,0,0,0,0,0,0,pi/3,pi/3]';
q = q_init;
robot_object.transformation_base_ = eye(4);
robot_object.CalculateFK(q);
p_eef = robot_object.frames_(1:3,4,14); % eef position
rotation_eef = robot_object.frames_(1:3,1:3,14); % eef orientation
p_t = p_eef + [0.05;0.05;0.05]; % target position
rotation_t = rotation_eef * RotationAxisAngle([1;0;0],pi/3) * RotationAxisAngle([0;1;0],pi/4); % target orientation
p_err = p_t - p_eef;
rotation_err = rotation_t * rotation_eef';
theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
%% resolved rates
figure(1)
%hold on
view(49,16)
camzoom(5)
axis equal
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);

index_movie = 0;
% iteration_steps is set to 1000 because this library is not for real-time
% control. For real-time control the iteration_steps should be re-defined.
iteration_steps = 0;

while (((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation)) && iteration_steps <=1000)
    iteration_steps = iteration_steps + 1;    
    robot_object.CalculateFK(q);
    p_eef = robot_object.frames_(1:3,4,14);
    rotation_eef = robot_object.frames_(1:3,1:3,14);
    % compute twist
    [twist_eef,p_err,theta_err] = ComputeTwist(p_t,p_eef,rotation_t,rotation_eef);
    % get Jacobian
    [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
    jacobian = jacobian_spherical;
    q_dot = pinv(jacobian)*twist_eef;
    q_dot_all = [0;0;0;0;0;q_dot];
    % update q
    q = q + q_dot_all *dt;
    %% update plot robot with frames
    % set flags for displaying individual frames, this serves as an example
    frame_to_display = [9, 10, 11, 12, 14];    
    % set link alpha value
    link_alpha_value = 0.5;
    % set frame scale factor
    frame_scale = 0.6;
    %draw robot and frames
    robot_object.DrawRobot(vertex_arm_origin, frame_to_display, frame_scale, link_alpha_value);
    axis([-0.8 0.8 -1.2 0.3 -0.3 0.9]) 
    drawnow;
    %%
    index_movie = index_movie + 1;
    movie_frames(index_movie) = getframe(gcf);
end