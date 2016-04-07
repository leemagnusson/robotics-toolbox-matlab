%% EEF workspace analysis
% Explore the robot with fixed RCM and move spherical arm to explore
% workspace
%%
clc
clear all
close all
load('urdf_info.mat');
load('vertex_arm_origin.mat');
load('arm_version_1.0.mat');
load('coupling_matrix.mat');
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
% input joint limits
joint_limits = [[-pi;pi] robot_kinematics(8).joint_limit_ robot_kinematics(11).joint_limit_ [-pi;pi] robot_kinematics(13).joint_limit_ robot_kinematics(14).joint_limit_];
if ~do_plot
    num_data = 1;
    sample_step = 10*pi/180;
    for q6 = joint_limits(1,1):(sample_step):joint_limits(2,1)
        for q7 = joint_limits(1,2):sample_step:joint_limits(2,2)
            for q8 = joint_limits(1,3):0.01:joint_limits(2,3)
                %             for q9 = jnt_limits(1,4):(6*sample_step):jnt_limits(2,4)
                %                 for q10 = jnt_limits(1,5):(6*sample_step):jnt_limits(2,5)
                q=[0,0,0,0,0,q6,q7,q8,0,0,0]';
                q_rcm = ConvertToRcm(q,coupling_matrix);
                frames = robot_kinematics.CalculateFK(q_rcm,transformation_base);
                eef_store(:,num_data) = frames(1:3,4,14);
                num_data = num_data + 1;
                %                 end
                %             end
            end
        end
    end
%     save('data/eef_workspace2.mat','eef_store');
else
    load('/data/eef_workspace2.mat');
    boundary_eef = boundary(eef_store',0.9);
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    trisurf(boundary_eef,eef_store(1,:)',eef_store(2,:)',eef_store(3,:)','Facecolor','red','FaceAlpha',0.3,'EdgeColor','none');
    axis([-1 1 -1 0.5 -0.2 1.2])
    grid on
    drawnow;
end