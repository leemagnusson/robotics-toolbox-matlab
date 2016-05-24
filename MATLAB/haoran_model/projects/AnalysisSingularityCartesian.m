%% Cartesian arm condition number analysis
% Explore the catesian workspace of the arm by controlling joint 1 to 6.
% Plot condition number with color map parula.
%%
clc
clear all
close all
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat')
load('arm_version_1.0.mat')
load('coupling_matrix.mat')
do_plot = 1; % 0 to resweep work space, 1 to just plot
robot_object.transformation_base_=eye(4);
joint_limits = [robot_object.joint_limit_(:,2) robot_object.joint_limit_(:,3) robot_object.joint_limit_(:,4) robot_object.joint_limit_(:,5) robot_object.joint_limit_(:,6) [-pi;pi]];
if ~do_plot
    num_data = 1;
    sample_step = 30*pi/180;
    for q1 = joint_limits(1,1):sample_step:joint_limits(2,1)
        for q2 = joint_limits(1,2):sample_step:joint_limits(2,2)
            for q3 = joint_limits(1,3):sample_step:joint_limits(2,3)
                for q4 = joint_limits(1,4):sample_step:joint_limits(2,4)
                    for q5 = joint_limits(1,5):sample_step:joint_limits(2,5)
                        %                         for q6 = joint_limits(1,6):(3*sample_step):joint_limits(2,6)
                        for q6 = 0
                            q=[q1,q2,q3,q4,q5,q6,0,0,0,0,0]';
                            robot_object.CalculateFK(q);
                            [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
                            jacobian_6DoF_car = jacobian_all(:,1:6);
                            [u1,e1,v1] = svd(jacobian_6DoF_car(1:3,:));
                            condition_number_translation(num_data) = e1(1,1)/e1(3,3);
                            [u2,e2,v2] = svd(jacobian_6DoF_car(4:6,:));
                            condition_number_rotation(num_data) = e2(1,1)/e2(3,3);
                            
                            rcm_store(:,num_data) = robot_object.frames_(1:3,4,7);
                            q_store(:,num_data) = q;
                            num_data = num_data + 1;
                        end
                    end
                end
            end
        end
    end
    save('../export/cartesian_singularity_6DoF.mat','rcm_store','condition_number_translation','condition_number_rotation','q_store');
else
    load('../export/cartesian_singularity_6DoF.mat');
    
    % setup arm
    q=[0,0,0,0,0,0,0,0,0,0,0]';
    robot_object.transformation_base_ = eye(4);
    robot_object.CalculateFK(q);
    condition_number_translation = 1 ./ condition_number_translation;
    
    figure(1)
    subplot(1,3,1)
    hold on
    view(90,0)
    axis equal
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    scatter3(rcm_store(1,:),rcm_store(2,:),rcm_store(3,:),4,condition_number_translation(:),'MarkerFaceColor','flat');
    colormap(parula)
%     colorbar
    grid on
    grid minor
    title('y-z')
    drawnow;
    
    subplot(1,3,2)
    hold on
    view(0,0)
    axis equal
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    scatter3(rcm_store(1,:),rcm_store(2,:),rcm_store(3,:),4,condition_number_translation(:),'MarkerFaceColor','flat');
    colormap(parula)
%     colorbar
    grid on
    grid minor
    title('x-z')
    drawnow;
    
    subplot(1,3,3)
    hold on
    view(0,90)
    axis equal
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    scatter3(rcm_store(1,:),rcm_store(2,:),rcm_store(3,:),4,condition_number_translation(:),'MarkerFaceColor','flat');
    colormap(parula)
    colorbar
    grid on
    grid minor
    title('x-y')
    drawnow;
    
    figure(2)
    
    % setup arm
    q=[0,0,0,0,0,0,0,0,0,0,0]';
    robot_object.transformation_base_ = eye(4);
    robot_object.CalculateFK(q);
    condition_number_rotation = 1 ./ condition_number_rotation;
    
    subplot(1,3,1)
    hold on
    view(90,0)
    axis equal
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    scatter3(rcm_store(1,:),rcm_store(2,:),rcm_store(3,:),4,condition_number_rotation(:),'MarkerFaceColor','flat');
    colormap(parula)
%     colorbar
    grid on
    grid minor
    title('y-z');
    drawnow;
    
    subplot(1,3,2)
    hold on
    view(0,0)
    axis equal
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    scatter3(rcm_store(1,:),rcm_store(2,:),rcm_store(3,:),4,condition_number_rotation(:),'MarkerFaceColor','flat');
    colormap(parula)
%     colorbar
    grid on
    grid minor
    title('x-z');
    drawnow;
    
    subplot(1,3,3)
    hold on
    view(0,90)
    axis equal
    robot_object.DrawRobot(vertex_arm_origin);
    hold on
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
    scatter3(rcm_store(1,:),rcm_store(2,:),rcm_store(3,:),4,condition_number_rotation(:),'MarkerFaceColor','flat');
    colormap(parula)
    colorbar
    grid on
    grid minor
    title('x-y');
    drawnow;
end