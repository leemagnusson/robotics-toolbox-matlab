%% RCM workspace analysis
% Explore the RCM workspace of the arm by controlling joint 1 to 5
% by Haoran Yu 3/30/2016
%%
clc
clear all
close all
load('urdf_info.mat')
load('vertex_arm_origin.mat')
load('arm_version_1.0.mat')
load('coupling_matrix.mat')
do_plot = 1; % 0 to resweep work space, 1 to just plot
figure(1)
hold on
view(46,23)
axis equal
% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
q_rcm = ConvertToRcm(q,coupling_matrix);
transformation_base=eye(4);
frames = robot_kinematics.CalculateFK(q_rcm,transformation_base);
arm_color = GetRobotColor(robot_kinematics);
DrawRobot(frames,vertex_arm_origin,arm_color);
hold on
joint_limits = [robot_kinematics(2).joint_limit_ robot_kinematics(3).joint_limit_ robot_kinematics(4).joint_limit_ robot_kinematics(5).joint_limit_ robot_kinematics(6).joint_limit_];
if ~do_plot
    num_data = 1;
    sample_step = 20*pi/180;
    for q1 = joint_limits(1,1):sample_step:joint_limits(2,1)
        for q2 = joint_limits(1,2):sample_step:joint_limits(2,2)
            for q3 = joint_limits(1,3):sample_step:joint_limits(2,3)
                for q4 = joint_limits(1,4):sample_step:joint_limits(2,4)
                    for q5 = joint_limits(1,5):sample_step:joint_limits(2,5)
                        q=[q1,q2,q3,q4,q5,0,0,0,0,0,0]';
                        q_rcm = ConvertToRcm(q,coupling_matrix);
                        frames = robot_kinematics.CalculateFK(q_rcm,transformation_base);
                        rcm_store(:,num_data) = frames(1:3,4,18);
                        num_data = num_data + 1;
                    end
                end
            end
        end
    end
%     save('/data_store/rcm_workspace.mat','rcm_store');
else
    load('/data/rcm_workspace.mat');
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