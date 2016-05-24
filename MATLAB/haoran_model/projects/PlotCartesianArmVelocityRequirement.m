%% Plot cartesian arm velocity requirement
% This script plot the velocity requirement for cartesian arm under the
% specification of two sets of requirements
% three joint angular velocity: joint_angular_velocity_max = [2.4 1.2 0.6];
% three max tip linear velocity: max_tip_linear = [0.1 0.15 0.2];
% the result is a table of 9 plots with each plot swept the workspace of
% the cartesian arm. Red means the cartesian arm could not satisfy the
% velocity requirement in this region and green means the cartesian arm
% could move in the velocity requirement.
%%
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
load('cartesian_max_joint_velocity_1.0.mat');
do_plot = 1; % 0 to resweep work space, 1 to just plot
figure(1)
% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
robot_object.transformation_base_ = eye(4);
robot_object.CalculateFK(q);
joint_angular_velocity_max = [2.4 1.2 0.6];
max_tip_linear = [0.1 0.15 0.2];
for index1 = 1:3
    for index2 = 1:3
        
        num_within_limit = 1;
        num_outof_limit = 1;
        clear rcm_outof_limit rcm_within_limit
        max_joint_velocity_cur = max_tip_linear(index2)/0.1 * max_joint_velocity_translation_store;
        subplot(3,3,(index1-1) * 3 + index2)
        robot_object.DrawRobot(vertex_arm_origin);
        hold on
        view(46,23)
        camzoom(1.5)
        axis equal
        hold on
        for index = 1 : length(rcm_store)
            if max_joint_velocity_cur(1,index) > joint_angular_velocity_max(index1) || max_joint_velocity_cur(2,index) > joint_angular_velocity_max(index1)...
                    || max_joint_velocity_cur(3,index) > joint_angular_velocity_max(index1) || max_joint_velocity_cur(4,index) > joint_angular_velocity_max(index1)...
                    || max_joint_velocity_cur(5,index) > joint_angular_velocity_max(index1) || max_joint_velocity_cur(6,index) > joint_angular_velocity_max(index1)...
                rcm_outof_limit(:,num_outof_limit) = rcm_store(:,index);
                plot3(rcm_outof_limit(1,num_outof_limit),rcm_outof_limit(2,num_outof_limit),rcm_outof_limit(3,num_outof_limit),'Color','red','Marker','.','MarkerSize',4);
                hold on
                num_outof_limit = num_outof_limit + 1;
            else
                rcm_within_limit(:,num_within_limit) = rcm_store(:,index);
                plot3(rcm_within_limit(1,num_within_limit),rcm_within_limit(2,num_within_limit),rcm_within_limit(3,num_within_limit),'Color','green','Marker','.','MarkerSize',4);
                hold on
                num_within_limit = num_within_limit + 1;
            end
            
        end
%         num_within_limit/ (num_within_limit + num_outof_limit)
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        axis([-0.5 0.5 -0.8 1 -1.2 1]);
        grid on
        grid minor
        drawnow;
    end
end