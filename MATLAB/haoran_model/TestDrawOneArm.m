%% Test the robot
% Plot the robot at different joint input
% can draw coordinate system
% by Haoran Yu 3/16/2016
%%
clc
clear all
close all
load('arm_version_1.0.mat')
load('vertex_arm_origin.mat')
load('coupling_matrix.mat')
arm_color = GetRobotColor(robot_kinematics);
transformation_base=eye(4);
figure(1)
hold on
view(62,28)
axis equal
for j=1:1
    cla
    q=[0,0,0,0,0,0,0,0,0,pi/2,pi/2]';
    q_rcm = ConvertToRcm(q,coupling_matrix);
    frames = robot_kinematics.CalculateFK(q_rcm,transformation_base);
    [jacobian_rcm,jacobian_cartesian,jacobian_all] = CalculateJacobianAll(frames);
%     rcm_rank = rank(J_rcm)
%     car_rank = rank(J_car)
%     all_rank = rank(J_all)
    DrawRobot(frames,vertex_arm_origin,arm_color,[],0.2);
    axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
end