%% Test the robot
% This code gives the user flexibility to test the robotics library. It
% serves as a reference code for other scripts.
% The features include:
% 1. setup the robot_object
% 2. test FK
% 3. Calculate jacobian
% 4. Calculate joint torque
% 5. Draw robot and coordinate system.
%%
clc
clear all
close all
load('arm_version_1.5.mat')
load('vertex_arm_origin_1.5.mat')
load('coupling_matrix.mat')
load('index_joints.mat')
robot_object.transformation_base_=eye(4);
figure(1)
hold on
view(-90,0)
axis equal
grid on
grid minor
for j=1
    cla
    q=[0,0,0,0,0,0,0,0,0,0,0]';
    robot_object.CalculateFK(q);
    [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
    robot_object.DrawRobot(vertex_arm_origin,[7:14], 0.15);
    DrawCoordinateSystem([0.15 0.15 0.15],eye(3),[0;0;0],'rgb','t')
    robot_object.InverseDynamics([0;0;9.81],zeros(11,1),zeros(11,1),'gravity');
    axis([ -0.3 0.3 -1 0.3 -0.4 0.9])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
end
