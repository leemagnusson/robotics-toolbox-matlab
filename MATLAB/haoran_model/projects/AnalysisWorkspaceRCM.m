%% RCM workspace analysis
% Explore the RCM workspace of the arm by controlling joint 1 to 5. This
% will tell us what the workspace looks like when docking the arm to
% trocars in the workspace. The code controls joint 1 to 5 within the
% workspace and record the position of the rcm link to point clouds.
% do_plot = 0: the data will be saved if setting
% do_plot = 1: will plot the workspace with the saved data
%%
clc
clear all
close all
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat')
load('arm_version_1.0.mat')
load('coupling_matrix.mat')
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
joint_limits = [robot_object.joint_limit_(:,2) robot_object.joint_limit_(:,3) robot_object.joint_limit_(:,4) robot_object.joint_limit_(:,5) robot_object.joint_limit_(:,6)];
if ~do_plot
    num_data = 1;
    sample_step = 20*pi/180;
    for q1 = joint_limits(1,1):sample_step:joint_limits(2,1)
        for q2 = joint_limits(1,2):sample_step:joint_limits(2,2)
            for q3 = joint_limits(1,3):sample_step:joint_limits(2,3)
                for q4 = joint_limits(1,4):sample_step:joint_limits(2,4)
                    for q5 = joint_limits(1,5):sample_step:joint_limits(2,5)
                        q=[q1,q2,q3,q4,q5,0,0,0,0,0,0]';
                        robot_object.CalculateFK(q);
                        rcm_store(:,num_data) = robot_object.frames_(1:3,4,18);
                        num_data = num_data + 1;
                    end
                end
            end
        end
    end
%     save('../export/rcm_workspace.mat','rcm_store');
else
    load('../export/rcm_workspace.mat');
    % filter work space
    num_filtered = 1;
    for i = 1 : length(rcm_store)
        if rcm_store(3,i)>0 && rcm_store(2,i)<0
            rcm_filtered(:,num_filtered) = rcm_store(:,i);
            num_filtered = num_filtered + 1;
        end
    end
    boundary_rcm = boundary(rcm_filtered');
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    trisurf(boundary_rcm,rcm_filtered(1,:)',rcm_filtered(2,:)',rcm_filtered(3,:)','Facecolor','red','FaceAlpha',0.1,'EdgeColor','none');
    axis([-1 1 -1 0.5 -0.2 1.2])
    grid on
    drawnow;
end