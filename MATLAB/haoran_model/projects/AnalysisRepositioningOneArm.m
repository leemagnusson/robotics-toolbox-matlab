%% Arm repositioning
% This is a script that reposition the arm with given constaint:
% Constaints:
% 1. End-effector position and orientation hold
% 2. RCM point position holds
% The simulation shows the arm reposition in 2 DOF with rotation around
% local x and y axis on the roll joint (joint 6)
%%
clc
clear all
close all
load('arm_version_1.0.mat')
load('vertex_arm_origin_1.0.mat')
load('coupling_matrix.mat')
robot_object.transformation_base_ = eye(4);
figure(1)
hold on
view(52,31)
axis equal
grid on 
grid minor
q=[0,0,0,0,0,0,0,0,0,0,0]';
% q=[0,0,pi/3,0,pi/4,0,-pi/4,0,0,0,0]';
dt = 0.001;
for j=1:1000
    cla
    
    robot_object.CalculateFK(q);
    [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
    jacobian_car_6DoF = robot_object.CalculateJacobian6DofRCM;
    robot_object.DrawRobot(vertex_arm_origin,[7],0.15);
%     omega_matrix = [eye(3) zeros(3,3); zeros(3,3) frames(1:3,1:3,7)];
    jacobian_temp = pinv(robot_object.frames_(1:3,1:3,7)) * jacobian_all(4:6,1:6);
    jacobian_spherical_rotation = jacobian_temp(1:2,:);
    if j <=50
    twist_repositioning = [zeros(9,1);1.5;0];
    title('Move around local x axis in positive direction')
    elseif j<=100
        twist_repositioning = [zeros(9,1);-1.5;0];
    title('Move around local x axis in negative direction')
    elseif j<=130
        twist_repositioning = [zeros(9,1);0;1.2];
    title('Move around local y axis in positive direction')
    else
        twist_repositioning = [zeros(9,1);0;-1];
    title('Move around local y axis in negative direction')
    end
    jacobian_repositioning = [jacobian_all;jacobian_car_6DoF(1:3,:) zeros(3,5);jacobian_spherical_rotation zeros(2,5)];
    q_dot = pinv(jacobian_repositioning) * twist_repositioning;
    q = q + q_dot*dt;
    axis([ -0.3 0.3 -1 0.3 -0.3 0.9])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
    movie_frames(j) = getframe(gcf);
    rcm_store(:,j) = robot_object.frames_(1:3,4,18);
    eef_store(:,j) = robot_object.frames_(1:3,4,14);
end