%% Plot spherical arm velocity requirement
% This script plot the velocity requirement for spherical arm under the
% specification of two sets of requirements
% three joint angular velocity: joint_angular_velocity_max = [2.4 1.2 0.6];
% three max tip linear velocity: max_tip_linear = [0.1 0.15 0.2];
% the result is a table of 9 plots with each plot swept the workspace of
% the spherical arm. Red means the spherical arm could not satisfy the
% velocity requirement in this region and green means the spherical arm
% could move in the velocity requirement.
%%
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('max_joint_velocity_1.0.mat');
do_plot = 1; % 0 to resweep work space, 1 to just plot
figure(1)
% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
robot_object.transformation_base_ = eye(4);
robot_object.CalculateFK(q);
% input joint limits
joint_limits = [[-pi;pi] robot_object.joint_limit_(:,8) robot_object.joint_limit_(:,11) [-pi;pi] robot_object.joint_limit_(:,13) robot_object.joint_limit_(:,14)];

angular_velocity_max = [2.4 1.2 0.6];
linear_velocity_max = 0.1;
max = [0.1 0.15 0.2];
for index1 = 1:3
    for index2 = 1:3
        
        num_within_limit = 1;
        num_outof_limit = 1;
        clear eef_outof_limit eef_within_limit
        max_joint_velocity_cur = max(index2)/0.03 * max_joint_velocity_store;
        subplot(3,3,(index1-1) * 3 + index2)
        robot_object.DrawRobot(vertex_arm_origin);
        hold on
        view(46,23)
        camzoom(2)
        axis equal
        hold on
        for index = 1 : length(eef_store)
            if max_joint_velocity_cur(1,index) > angular_velocity_max(index1) || max_joint_velocity_cur(2,index) > angular_velocity_max(index1)
                eef_outof_limit(:,num_outof_limit) = eef_store(:,index);
                plot3(eef_outof_limit(1,num_outof_limit),eef_outof_limit(2,num_outof_limit),eef_outof_limit(3,num_outof_limit),'Color','red','Marker','.','MarkerSize',4);
                hold on
                num_outof_limit = num_outof_limit + 1;
            else
                eef_within_limit(:,num_within_limit) = eef_store(:,index);
                plot3(eef_within_limit(1,num_within_limit),eef_within_limit(2,num_within_limit),eef_within_limit(3,num_within_limit),'Color','green','Marker','.','MarkerSize',4);
                hold on
                num_within_limit = num_within_limit + 1;
            end
        end
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        axis([-0.3 0.3 -1 0.2 -0.2 0.75])
        grid on
        drawnow;
    end
end