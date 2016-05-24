%% Collision detection simulation
% This simulation reads the logs of hernia on simulation data and detect
% the collision between arms. This analysis first import the the joint
% values for hernia setup. The joint values for hernia setup is calculated
% from "AnalysisHerniaProcedureSetup.m" and saved in "q_init_setup_hernia.mat".
% The init condition for trocar position, target position and table adapter
% position are from "InitHerniaSetup.m". The log file is input from
% "InitLogToolPath.m" and saved in "hernia_tool_path.mat". The vertex data
% are stls models. "vertex_bed.mat" and "vertex_bed_adapter.mat" have table
% and table adapter information in them.
% set is_artificial = 0 if want to load the real tool path
% set is_artificial = 1 if want to create artificial path that the arm will
% collide
%%
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat')
load('point_boundary_arm_1.0.mat');
load('joint_limit_active_1.0.mat');
load('vertex_patient_body.mat');
load('q_init_setup_hernia.mat');
load('hernia_tool_path_raw.mat');
load('index_joints.mat');
load('coupling_matrix.mat');
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
load('point_clouds_hernia_workspace.mat');
InitHerniaSetup;
% iterative inverse kinematics parameters
InitIKParameters;

% set simulation
is_artificial = 0; % set to 1 to create artificial colliding simulation
% figure parameters
figure_handle = figure(1);
set(figure_handle,'Position',[600 10 750 750])
hold on
axis equal
view(0,40)
% camzoom(3)
% set index
movie_index = 1;
num_collision = 0;
num_q_store = 0;
index_start = 1;
index_end = 29992;
% index_start = 1;
% index_end = 29992;
sample_rate = 1; % change the sample rate for simulation
do_plot = 1; % set to 1 if want to plot
do_save = 1; % set to 1 if want to store joint values
do_save_video = 1; % set to 1 if want to save video
do_check_collision = 1; % set to 1 if want to check collision

% selected_bed_adapter = [2 1 5];
index_robot = 0;
for index_bed_adapter = selected_bed_adapter;
    if index_bed_adapter ~=0 && index_bed_adapter<=8
        index_robot = index_robot + 1;
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
        transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    end
end
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
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
total_data = 1;
down_sample = 10;
while (total_data<=10)
    total_data
    index_point_clouds = randi([1 length(point_clouds_hernia_workspace)],1,1);
    p_target_region = point_clouds_hernia_workspace(:,index_point_clouds);
    
    for switch_arm = 1 : 2
        p_init = p_eef;
        rotation_init = rotation_eef;
        distance_to_target = randi([30 70],1,1)/1000;
        if distance_to_target >= norm(translation_trocar(:,2) - p_target_region)
            p_target_camera = p_target_region;
        else
            p_target_camera = p_target_region + (translation_trocar(:,2) - p_target_region) / norm(translation_trocar(:,2) - p_target_region) * distance_to_target;
        end
        for index_sample = 1 : sample_rate : down_sample
            if switch_arm == 1
                % set the target pose for arm 1 and arm 3.
                p_t(:,1) = index_sample * (p_target_region - p_init(:,1)) / down_sample + p_init(:,1);
                p_t(:,3) = index_sample * (p_target_region - p_init(:,3)) / down_sample + p_init(:,3);
                p_t(:,2) = p_init(:,2);
                rotation_t(:,:,1) = rotation_init(:,:,1);
                rotation_t(:,:,2) = rotation_init(:,:,2);
                rotation_t(:,:,3) = rotation_init(:,:,3);
            else
                % set the target pose for arm 1 and arm 3.
                p_t(:,2) = index_sample * (p_target_camera - p_init(:,2)) / down_sample + p_init(:,2);
                p_t(:,1) = p_init(:,1);
                p_t(:,3) = p_init(:,3);
                rotation_t(:,:,1) = rotation_init(:,:,1);
                rotation_t(:,:,2) = rotation_init(:,:,2);
                rotation_t(:,:,3) = rotation_init(:,:,3);
            end
            
            % move arm 1 and arm 3
            for index = 1:3
                q(:,index) = robot_arms{index}.InverseKinematics(q(:,index),p_t(:,index),rotation_t(:,:,index),'Spherical 6');
                p_eef(:,index) = robot_arms{index}.frames_(1:3,4,index_eef);
                rotation_eef(:,:,index) = robot_arms{index}.frames_(1:3,1:3,index_eef);
            end
            
            % transform point clouds boundary box
            for index = 1:3
                robot_arms{index}.CalculateFK(q(:,index));
                for i = 7:10
                    rotation = robot_arms{index}.frames_(1:3,1:3,i);
                    translation = robot_arms{index}.frames_(1:3,4,i);
                    for index_boundary = 1 : length(point_boundary_arm{i})
                        point_boundary_transformed{i,index}(index_boundary,:) = (rotation * point_boundary_arm{i}(index_boundary,:)' + translation)'; % transformed point clouds stored separately
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
                for index_arm_save = 1 : 3
                    q_store{index_arm_save}(:,num_q_store) = q(:,index_arm_save);
                    for index_joint_number_limit = 1 :  length(jnt_limit_active)
                        joint_percentage{index_arm_save}(index_joint_number_limit,num_q_store) = (q(index_joint_number_limit,index_arm_save) - jnt_limit_active(1,index_joint_number_limit)) / (jnt_limit_active(2,index_joint_number_limit) - jnt_limit_active(1,index_joint_number_limit));
                    end
                end
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
                DrawCoordinateSystem([0.02 0.02 0.02],rotation_camera,translation_camera,'rgb','c')
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
                vertex_hernia_workspace_transformed = transformSTL(vertex_hernia_macro_workspace,rotation_setup,translation_setup);
                rgba = [1 0 0 0.3];
                PlotStl(vertex_hernia_workspace_transformed,rgba);
                hold on
                axis([-1 1 -0.8 0.8 -0.1 1.7])
                light('Position',[1 3 2]);
                light('Position',[-3 -1 -3]);
                drawnow;
            end
            if do_save_video
                movie_frames(movie_index) = getframe(gcf);
                movie_index = movie_index + 1;
            end
        end
    end
    total_data = total_data + 1;
end
save('..\export\hernia_macro_collision_data.mat','q_store','joint_percentage','q_store_collision','collision_store')