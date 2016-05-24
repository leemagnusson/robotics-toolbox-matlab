%% Collision detection simulation
% This simulation reads the logs of suture on simulation data and detect
% the collision between arms. This analysis first import the the joint
% values for hernia setup. The joint values for hernia setup is calculated
% from "AnalysisHerniaProcedureSetup.m" and saved in "q_init_setup_hernia.mat".
% The init condition for trocar position, target position and table adapter
% position are from "InitHerniaSetup.m". The log file is input from
% "ImportSutureToolPath.m" and saved in "suture_tool_path_raw.mat". The vertex data
% are stls models. "vertex_bed.mat" and "vertex_bed_adapter.mat" have table
% and table adapter information in them.
% set is_artificial = 0 if want to load the real tool path
% set is_artificial = 1 if want to create artificial path that the arm will
% collide
%%
% clc
% clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('point_boundary_arm_1.0.mat');
load('vertex_patient_body.mat');
load('q_init_setup_hernia.mat');
% load('sponge_tool_path_raw.mat');
% load('suture_tool_path_raw.mat');
% load('sponge_tool_path_raw_with_angle.mat');
% load('sponge_tool_path_2016_5_5.mat');
% load('egg_sponge_user04_tool_path_raw_with_angle.mat');

load('index_joints.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
InitHerniaSetup;
% iterative inverse kinematics parameters
InitIKParameters;
% set simulation
is_artificial = 0; % set to 1 to create artificial colliding simulation

robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};

% figure parameters
figure_handle = figure(1);
set(figure_handle,'Position',[600 10 750 750])
hold on
axis equal
view(3)
view(0,65)
camzoom(5)
% set index
movie_index = 1;
num_collision = 0;
num_q_store = 0;
index_start = 1;
index_end = length(time_log);
if choose_save_video
    sample_rate = 200; % change the sample rate for simulation
    do_plot = 1; % set to 1 if want to plot
    do_save = 0; % set to 1 if want to store joint values
    do_save_video = 1; % set to 1 if want to save video
    do_check_collision = 0; % set to 1 if want to check collision
else
    sample_rate = 1; % change the sample rate for simulation
    do_plot = 0; % set to 1 if want to plot
    do_save = 1; % set to 1 if want to store joint values
    do_save_video = 0; % set to 1 if want to save video
    do_check_collision = 0; % set to 1 if want to check collision
end
index_robot = 0;
% selected_bed_adapter = [2 1 5];
for index_bed_adapter = selected_bed_adapter;
    if index_bed_adapter ~=0 && index_bed_adapter <=8
        index_robot = index_robot + 1;
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
        transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    end
end

% initialze all the arms
for index_robot = 1:3
    q(:,index_robot) = q_init_setup(:,index_robot);
    robot_arms{index_robot}.transformation_base_ = transformation_base(:,:,index_robot);
    robot_arms{index_robot}.CalculateFK(q(:,index_robot));
    p_eef(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_eef);
    rotation_eef(:,:,index_robot) = robot_arms{index_robot}.frames_(1:3,1:3,index_eef);
end

% setup bed adapter
% q_bed_adapter = [0 -0.02 0;-40*pi/180 -0.18 0;0 -0.02 0]';
index_robot = 0;

for index_sample = index_start : sample_rate : index_end
    index_sample
    % transform the tool path on all the arms into world frame
    tool_path_left(1:3,4,index_sample) = rotation_camera * tool_path_left(1:3,4,index_sample) + translation_camera;
    tool_path_left(1:3,1:3,index_sample) = rotation_camera * tool_path_left(1:3,1:3,index_sample);
    if is_artificial
        tool_path_right(1:3,4,index_sample) = rotation_camera * tool_path_right(1:3,4,1) + translation_camera + [-0.00025*(index_sample-1767);0;0];
    else
        tool_path_right(1:3,4,index_sample) = rotation_camera * tool_path_right(1:3,4,index_sample) + translation_camera;
    end
    tool_path_right(1:3,1:3,index_sample) = rotation_camera * tool_path_right(1:3,1:3,index_sample);
    tool_path_camera(1:3,4,index_sample) = rotation_camera * tool_path_camera(1:3,4,index_sample) + translation_camera;
    tool_path_camera(1:3,1:3,index_sample) = rotation_camera * tool_path_camera(1:3,1:3,index_sample);
    
    % set the target pose for arm 1 and arm 3.
    p_t(:,1) = tool_path_left(1:3,4,index_sample);
    p_t(:,2) = tool_path_camera(1:3,4,index_sample);
    p_t(:,3) = tool_path_right(1:3,4,index_sample);
    
    rotation_t(:,:,1) = tool_path_left(1:3,1:3,index_sample);
    rotation_t(:,:,2) = tool_path_camera(1:3,1:3,index_sample);
    rotation_t(:,:,3) = tool_path_right(1:3,1:3,index_sample);
    
    % move arm 1 and arm 3 with IK
    for index_robot = [1 3]
        q(:,index_robot) = robot_arms{index_robot}.InverseKinematics(q(:,index_robot),p_t(:,index_robot),rotation_t(:,:,index_robot),'Spherical 6');
    end
    
    % transform point clouds boundary box
    for index_robot = 1:3
        robot_arms{index_robot}.CalculateFK(q(:,index_robot));
        for i = 7:10
            rotation = robot_arms{index_robot}.frames_(1:3,1:3,i);
            translation = robot_arms{index_robot}.frames_(1:3,4,i);
            for index_boundary = 1 : length(point_boundary_arm{i})
                point_boundary_transformed{i,index_robot}(index_boundary,:) = (rotation * point_boundary_arm{i}(index_boundary,:)' + translation)'; % transformed point clouds stored separately
            end
        end
    end
    
    arm_color1 = robot_arms{1}.color_;
    arm_color2 = robot_arms{2}.color_;
    arm_color3 = robot_arms{3}.color_;
    
    if do_check_collision
        % detect collision
        [collision12,colliding_arm_index12] = FindCollisionTwoRobot (point_boundary_transformed(:,1),point_boundary_transformed(:,2),index_car:index_pitch_c);
        [collision13,colliding_arm_index13] = FindCollisionTwoRobot (point_boundary_transformed(:,1),point_boundary_transformed(:,3),index_car:index_pitch_c);
        [collision23,colliding_arm_index23] = FindCollisionTwoRobot (point_boundary_transformed(:,2),point_boundary_transformed(:,3),index_car:index_pitch_c);
        
        % change arm color for colliding links
        [arm_color1,arm_color2] = ChangeLinkColor(arm_color1,arm_color2,collision12,colliding_arm_index12);
        [arm_color1,arm_color3] = ChangeLinkColor(arm_color1,arm_color3,collision13,colliding_arm_index13);
        [arm_color2,arm_color3] = ChangeLinkColor(arm_color2,arm_color3,collision23,colliding_arm_index23);
        robot_arms{1}.color_ = arm_color1;
        robot_arms{2}.color_ = arm_color2;
        robot_arms{3}.color_ = arm_color3;
    end
    
    % save joint values
    if do_save
        num_q_store = num_q_store + 1;
        q_store{1}(:,num_q_store) = q(:,1);
        q_store{2}(:,num_q_store) = q(:,2);
        q_store{3}(:,num_q_store) = q(:,3);
        if do_check_collision
            if collision12 || collision13 || collision23
                num_collision = num_collision + 1;
                q_store_collision(:,:,num_collision) = q;
                collision_store(:,num_collision) = [collision12;collision13;collision23];
            end
        end
    end
    
    % plot robot
    if do_plot
        cla
        DrawCoordinateSystem([0.02 0.02 0.02],tool_path_camera(1:3,1:3,index_sample),tool_path_camera(1:3,4,index_sample),'rgb','c')
        hold on
        DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
        hold on
        robot_arms{1}.DrawRobot(vertex_arm_origin)
        hold on
        robot_arms{2}.DrawRobot(vertex_arm_origin)
        hold on
        robot_arms{3}.DrawRobot(vertex_arm_origin)
        hold on
        frames_bed_adapter1 = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,selected_bed_adapter(1)));
        frames_bed_adapter2 = CalculateBedAdapterFK(q_bed_adapter(:,2),frames_bed_adapter_base(:,:,selected_bed_adapter(2)));
        frames_bed_adapter3 = CalculateBedAdapterFK(q_bed_adapter(:,3),frames_bed_adapter_base(:,:,selected_bed_adapter(3)));
        DrawBed(vertex_bed,[0.0 0.7 0.0 1])
        hold on
        DrawBedAdapter(frames_bed_adapter1,vertex_bed_adapter,[1 0 0 1])
        hold on
        DrawBedAdapter(frames_bed_adapter2,vertex_bed_adapter,[1 0 0 1])
        hold on
        DrawBedAdapter(frames_bed_adapter3,vertex_bed_adapter,[1 0 0 1])
        hold on
        
        if do_check_collision
            if collision12 || collision13 || collision23
                title(['Collision12 =' num2str(collision12) ';   ' 'Collision13 =' num2str(collision13) ';   ' 'Collision23 =' num2str(collision23)],'Color','r')
            else
                title(['Collision12 =' num2str(collision12) ';   ' 'Collision13 =' num2str(collision13) ';   ' 'Collision23 =' num2str(collision23)],'Color','b')
            end
        end
        vertex_patient_body_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
        rgba = [0 0 1 0.1];
        PlotStl(vertex_patient_body_transformed,rgba);
        hold on
        axis([-1.2 1 -0.8 0.8 -0.2 2.8]);
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        drawnow;
    end
    if do_save_video
        movie_frames(movie_index) = getframe(gcf);
        movie_index = movie_index + 1;
    end
end

sample_first_index = index_start;
sample_last_index = index_sample;
if do_save
%     % calculate the time separation
%     dt_separation = (time_log(sample_last_index) - time_log(sample_first_index))/(sample_last_index - sample_first_index)*sample_rate;
%     time_separation = 0 : dt_separation : (sample_last_index - sample_first_index)/sample_rate * dt_separation;
%     % calculate qd
%     for i = 1 : 3
%         for j = 1 : 11
%             qd_store{i}(j,:) = diff(q_store{i}(j,:))/dt_separation;
%         end
%     end
%     % calculate qdd
%     for i = 1 : 3
%         for j = 1 : 11
%             qdd_store{i}(j,:) = diff(qd_store{i}(j,:))/dt_separation;
%         end
%     end
%     figure(2)
%     index_arm = 3;
%     plot(time_separation,q_store{index_arm}(6:11,:));
%     ylabel('q(rad)');
%     xlabel('t(s)');
%     legend('roll','pitch','trans','rotate','wr','dis_wr')
%     title('joint angle')
%     figure(3)
%     plot(time_separation(1:length(time_separation)-1),qd_store{index_arm}(6:11,:));
%     ylabel('qd(rad/s)');
%     xlabel('t(s)');
%     legend('roll','pitch','trans','rotate','wr','dis_wr')
%     title('joint velocity')
%     figure(4)
%     plot(time_separation(1:length(time_separation)-2),qdd_store{index_arm}(6:11,:));
%     ylabel('qdd(rad/s2)');
%     xlabel('t(s)');
%     legend('roll','pitch','trans','rotate','wr','dis_wr')
%     title('joint acceleration')
    tool_drive{1} = [q_store{1}(8:11,:);gripper_left(:,index_start : sample_rate : index_end)];
    tool_drive{2} = [q_store{2}(8:11,:);gripper_camera(:,index_start : sample_rate : index_end)];
    tool_drive{3} = [q_store{3}(8:11,:);gripper_right(:,index_start : sample_rate : index_end)];
%     data_name2 = 'export\egg_sponge_user04_joint_log_hernia_setup_with_angle.mat';
%     data_name3 = 'export\egg_sponge_user04_tool_drive';
%     export(data_name2,'q_store','qd_store','qdd_store','time_log');
    export(data_name2,'q_store','time_log');
    export(data_name3,'tool_drive','time_log');
end