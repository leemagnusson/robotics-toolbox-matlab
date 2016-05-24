%% Touch point simulation
% This code simulates the touch point by giving 
%%
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('init_joint_state_1.0.mat');
load('vertex_patient_body.mat');
load('q_init_setup_hernia.mat');
load('index_joints.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
% iterative inverse kinematics parameters
InitIKParameters;
% set simulation
is_artificial = 1; % set to 1 to create artificial colliding simulation
% figure parameters
figure_handle = figure(1);
set(figure_handle,'Position',[600 10 750 750])
hold on
axis equal
view(-115,24)
camzoom(2)
% set index
movie_index = 1;
dt = dt/sqrt(10);
do_plot = 1; % set to 1 if want to plot
do_save_video = 1; % set to 1 if want to save video

frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,2));
transformation_base = frames_bed_adapter(:,:,end);


% initialze all the arms
q = q_retracted(:,1);
robot_object.transformation_base_ = transformation_base;
robot_object.CalculateFK(q);
p_eef = robot_object.frames_(1:3,4,index_eef);
rotation_eef = robot_object.frames_(1:3,1:3,index_eef);

index_robot = 1;
torque_to_acc = 2000;

force1 = [0;-7.5;7.5];
% force2 = [0;-7.5;7.5];
force2 = [0;-7.5;15];
for index_sample = 1:200
    p1 = robot_object.frames_(1:3,4,3) - robot_object.frames_(1:3,3,3) * 0.1;
    p2 = robot_object.frames_(1:3,4,5) + robot_object.frames_(1:3,3,5) * 0.25;
    if do_plot
        cla
        DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
        hold on
        robot_object.DrawRobot(vertex_arm_origin)
        hold on
        plot3(p1(1),p1(2),p1(3),'MarkerSize',20,'Marker','*')
        hold on
        plot3(p2(1),p2(2),p2(3),'MarkerSize',20,'Marker','*')
        hold on
        mArrow3(p1, p1 + force1/50);
        hold on
        mArrow3(p2, p2 + force2/50);
        hold on
        frames_bed_adapter1 = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,selected_bed_adapter(1)));
        DrawBed(vertex_bed,[0.0 0.7 0.0 1])
        hold on
        DrawBedAdapter(frames_bed_adapter1,vertex_bed_adapter,[1 0 0 1])
        hold on
        axis([-1.2 1 -1.2 0.4 -0.2 2.1])
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        drawnow;
    end
    if do_save_video
        movie_frames(movie_index) = getframe(gcf);
        movie_index = movie_index + 1;
    end
    
    jacobian_1 = robot_object.CalculateJacobianGeneral(p1,3);
    jacobian_2 = robot_object.CalculateJacobianGeneral(p2,5);
    tau_1 = jacobian_1(1:3,:)' * force1;
    tau_2 = jacobian_2(1:3,:)' * force2;
    tau_1_full = [tau_1;zeros(11-length(tau_1),1)];
    tau_2_full = [tau_2;zeros(11-length(tau_2),1)];
    tau_full = tau_1_full + tau_2_full;
    acc_full = torque_to_acc * tau_full;
    q = q + acc_full * dt * dt;
    robot_object.CalculateFK(q);
end



force1 = [10;10;10];
moment1_scale = 6;
for index_sample = 1:150
    p1 = robot_object.frames_(1:3,4,5) + robot_object.frames_(1:3,3,5) * 0.25;
    moment1 = moment1_scale * robot_object.frames_(1:3,3,5);
    % plot robot
    if do_plot
        cla
        DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
        hold on
        robot_object.DrawRobot(vertex_arm_origin)
        hold on
        plot3(p1(1),p1(2),p1(3),'MarkerSize',20,'Marker','*')
        hold on
        mArrow3(p1, p1 + force1/50);
        hold on
        mArrow3(p1, p1 + moment1/10,'color','r');
        hold on
        frames_bed_adapter1 = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,selected_bed_adapter(1)));
        DrawBed(vertex_bed,[0.0 0.7 0.0 1])
        hold on
        DrawBedAdapter(frames_bed_adapter1,vertex_bed_adapter,[1 0 0 1])
        hold on
        axis([-1.2 1 -1.2 0.4 -0.2 2.1])
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        drawnow;
    end
    if do_save_video
        movie_frames(movie_index) = getframe(gcf);
        movie_index = movie_index + 1;
    end
    
    jacobian_1 = robot_object.CalculateJacobianGeneral(p1,5);
    tau_1 = jacobian_1' * [force1;moment1];
    tau_1_full = [tau_1;zeros(11-length(tau_1),1)];
    tau_2_full = zeros(11,1);
    tau_full = tau_1_full + tau_2_full;
    acc_full = torque_to_acc * tau_full;
    q = q + acc_full * dt * dt;
    robot_object.CalculateFK(q);
end
