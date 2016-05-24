%% EEF workspace analysis
% Explore the robot with fixed RCM and move spherical arm to explore
% workspace. This code control joint 6 7 and 8 within the joint limits
% to sweep the workspace of the spherical arm and record the position of
% the eef to point clouds.
% do_plot = 0: the data will be saved if setting
% do_plot = 1: will plot the workspace with the saved data
%%
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
do_plot = 1; % 0 to resweep work space, 1 to just plot
figure(1)
hold on
view(46,23)
axis equal
% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
robot_object.transformation_base_=eye(4);
robot_object.CalculateFK(q);
robot_object.DrawRobot(vertex_arm_origin);
hold on
% input joint limits
joint_limits = [[-pi;pi] robot_object.joint_limit_(:,8) robot_object.joint_limit_(:,11) [-pi;pi] robot_object.joint_limit_(:,13) robot_object.joint_limit_(:,14)];
if ~do_plot
    num_data = 1;
    sample_step = 10*pi/180;
    for q6 = joint_limits(1,1):(sample_step):joint_limits(2,1)
        for q7 = joint_limits(1,2):sample_step:joint_limits(2,2)
            for q8 = joint_limits(1,3):0.01:joint_limits(2,3)
                %             for q9 = jnt_limits(1,4):(6*sample_step):jnt_limits(2,4)
                %                 for q10 = jnt_limits(1,5):(6*sample_step):jnt_limits(2,5)
                q=[0,0,0,0,0,q6,q7,q8,0,0,0]';
                robot_object.CalculateFK(q);
                eef_store(:,num_data) = robot_object.frames_(1:3,4,14);
                num_data = num_data + 1;
                %                 end
                %             end
            end
        end
    end
%     save('../export/eef_workspace2.mat','eef_store');
else
    load('../export/eef_workspace2.mat');
    boundary_eef = boundary(eef_store',0.9);
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    trisurf(boundary_eef,eef_store(1,:)',eef_store(2,:)',eef_store(3,:)','Facecolor','red','FaceAlpha',0.3,'EdgeColor','none');
    axis([-1 1 -1 0.5 -0.2 1.2])
    grid on
    drawnow;
end