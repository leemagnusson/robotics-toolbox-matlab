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
figure(1)
hold on
view(46,23)
axis equal
% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
robot_object.transformation_base_ = eye(4);
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
                [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
                %                 jacobian = jacobian_spherical(1:3,1:3);
                [u,e,v] = svd(jacobian_spherical);
                sigma_max = e(1,1);
                sigma_min = e(end,end);
                condition_number(num_data) = sigma_max/sigma_min;
                eef_store(:,num_data) = robot_object.frames_(1:3,4,14);
                q_store(:,num_data) = q;
                num_data = num_data + 1;
                %                 end
                %             end
            end
        end
    end
    save('../export/eef_singularity_6DoF.mat','eef_store','condition_number','q_store');
else
    load('../export/eef_singularity_6DoF_denser.mat');
    condition_number = 1 ./ condition_number;
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1 1 -1 0.5 -0.2 1.2]);
    scatter3(eef_store(1,:),eef_store(2,:),eef_store(3,:),4,condition_number(:),'MarkerFaceColor','flat');
    colormap(parula)
    colorbar
    grid on
    grid minor
    drawnow;
end