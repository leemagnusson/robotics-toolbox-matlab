%% EEF singularity analysis
% Explore the robot with fixed RCM and move spherical arm to sweep
% workspace. Plot condition number with color map parula.
%%
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
do_plot = 1; % 0 to resweep work space, 1 to just plot

% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
robot_object.transformation_base_ = eye(4);
robot_object.CalculateFK(q);
% h1 = figure(1);
% hold on
% view(46,23)
% axis equal

num_data = 1;
v_max = 0.1;
joint_velocity_max = 2;
figure_num = 0;
% h2 = figure(2);
h = figure(1);
% input joint limits
joint_limits = [[-pi;pi] robot_object.joint_limit_(:,8) robot_object.joint_limit_(:,11) [-pi;pi] robot_object.joint_limit_(:,13) robot_object.joint_limit_(:,14)];
for q7 =  [joint_limits(1,2), (3*joint_limits(1,2) + 1*joint_limits(2,2))/4, (2*joint_limits(1,2) + 2*joint_limits(2,2))/4, (1*joint_limits(1,2) + 3*joint_limits(2,2))/4, joint_limits(2,2)]
    num_data = 1;
    for q8 = joint_limits(1,3):0.002:joint_limits(2,3)

        q=[0,0,0,0,0,0,q7,q8,0,0,0]';
        robot_object.CalculateFK(q);
        [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
        [u,e,v] = svd(jacobian_spherical(1:3,:));
        q_dot1 = pinv(jacobian_spherical) * [v_max * u(:,1);zeros(3,1)];
        q_dot2 = pinv(jacobian_spherical) * [v_max * u(:,2);zeros(3,1)];
        q_dot3 = pinv(jacobian_spherical) * [v_max * u(:,3);zeros(3,1)];
        q_dot = [q_dot1 q_dot2 q_dot3];
        q_dot_max(:,num_data) = [max(abs(q_dot(1,:)));max(abs(q_dot(2,:)));max(abs(q_dot(3,:)));max(abs(q_dot(4,:)));max(abs(q_dot(5,:)));max(abs(q_dot(6,:)))];
        tool_length(num_data) = norm(robot_object.frames_(1:3,4,14)-robot_object.frames_(1:3,4,18));
        num_data = num_data + 1; 
    end
    figure_num = figure_num + 1;
    subplot(2,3,figure_num);
    plot(tool_length,q_dot_max([1 2 4 5 6],:))
    hold on
    plot(tool_length, joint_velocity_max * ones(length(tool_length)),'Color','black');
    legend('roll','pitch','tool rotate','wrist 1','wrist 2');
    xlabel('tool tip to trocar distance (m)')
    ylabel('joint velocity (rad/s)')
    title(strcat('Pitch arm at configuration',num2str(figure_num)));
    axis([0.03 0.2 0 3.5])
    grid on 
    grid minor
    subplot(2,3,6)
    view(46,23)
    axis equal
    q=[0,0,0,0,0,0,q7,0,0,0,0]';
    robot_object.CalculateFK(q);
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],robot_object.frames_(1:3,1:3,14),robot_object.frames_(1:3,4,14),'rgb',num2str(figure_num))
            hold on
end
subplot(2,3,6)
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
grid on
grid minor
drawnow;
set(0,'currentfigure',h)
title('Straight wrist velocity analysis');