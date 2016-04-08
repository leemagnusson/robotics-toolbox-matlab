%% Collision detection simulation
% This simulation reads the logs of hernia on simulation data and detect
% the collision between arms.
% set is_artificial = 0 if want to load the real tool path
% set is_artificial = 1 if want to create artificial path that the arm will
% collide
%%
clc
clear all
close all
load('urdf_info.mat');
load('vertex_arm_origin.mat');
load('vertex_hernia_patient_body.mat');
load('arm_version_1.0.mat')
load('q_init_setup_hernia.mat');
load('hernia_tool_path.mat');
load('point_boundary_arm.mat');
load('index_joints.mat');
load('coupling_matrix.mat');
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
view(3)
view(-2,43)

% set index
movie_index = 1;
num_collision = 0;
num_q_store = 0;
index_start = 1767;
index_end = 2592;
sample_rate = 100; % change the sample rate for simulation
do_plot = 1; % set to 1 if want to plot
do_save = 0; % set to 1 if want to store joint values
do_save_video = 0; % set to 1 if want to save video

% initialze all the arms
q(:,1) = q_init_setup(:,1);
q_rcm(:,1) = ConvertToRcm(q_init_setup(:,1),coupling_matrix);
frames_robot1 = robot_kinematics.CalculateFK(q_rcm(:,1),transformation_base(:,:,1));
p_eef(:,1) = frames_robot1(1:3,4,index_eef);
rotation_eef(:,:,1) = frames_robot1(1:3,1:3,index_eef);

q(:,2) = q_init_setup(:,2);
q_rcm(:,2) = ConvertToRcm(q_init_setup(:,2),coupling_matrix);
frames_robot2 = robot_kinematics.CalculateFK(q_rcm(:,2),transformation_base(:,:,2));
p_eef(:,2) = frames_robot2(1:3,4,index_eef);
rotation_eef(:,:,2) = frames_robot2(1:3,1:3,index_eef);

q(:,3) = q_init_setup(:,3);
q_rcm(:,3) = ConvertToRcm(q_init_setup(:,3),coupling_matrix);
frames_robot3 = robot_kinematics.CalculateFK(q_rcm(:,3),transformation_base(:,:,3));
p_eef(:,3) = frames_robot3(1:3,4,index_eef);
rotation_eef(:,:,3) = frames_robot3(1:3,1:3,index_eef);

for index_sample = index_start : sample_rate : index_end
    % transform the tool path on all the arms into world frame
    tool_path_left(1:3,4,index_sample) = rotation_camera * tool_path_left(1:3,4,1) + translation_camera;
    tool_path_left(1:3,1:3,index_sample) = rotation_camera * tool_path_left(1:3,1:3,index_sample);
    if is_artificial
        tool_path_right(1:3,4,index_sample) = rotation_camera * tool_path_right(1:3,4,1) + translation_camera + [-0.00025*(index_sample-1767);0;0];
    else
        tool_path_right(1:3,4,index_sample) = rotation_camera * tool_path_right(1:3,4,index_sample) + translation_camera;
    end
    tool_path_right(1:3,1:3,index_sample) = rotation_camera * tool_path_right(1:3,1:3,index_sample);
    
    % set the target pose for arm 1 and arm 3.
    p_t(:,1) = tool_path_left(1:3,4,index_sample);
    p_t(:,3) = tool_path_right(1:3,4,index_sample);
    
    rotation_t(:,:,1) = tool_path_left(1:3,1:3,index_sample);
    rotation_t(:,:,3) = tool_path_right(1:3,1:3,index_sample);
    
    % move arm 1 and arm 3
    for index = [1 3]
        p_err = p_t(:,index) - p_eef(:,index);
        rotation_err = rotation_t(:,:,index) * rotation_eef(:,:,index)';
        theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
        % iterative IK
        while((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation))
            % compute the current frames
            frames_cur = robot_kinematics.CalculateFK(q_rcm(:,index),transformation_base(:,:,index));
            % compute eef pose
            p_eef(:,index) = frames_cur(1:3,4,index_eef);
            rotation_eef(:,:,index) = frames_cur(1:3,1:3,index_eef);
            % compute twist
            [twist_eef,p_err,theta_err] = ComputeTwist(p_t(:,index),p_eef(:,index),rotation_t(:,:,index),rotation_eef(:,:,index));
            % compute Jacobian
            [jacobian_rcm,jacobian_cartesian,jacobian_all] = CalculateJacobianAll(frames_cur);
            % compute joint velocity
            q_dot = pinv(jacobian_rcm)*twist_eef;
            q_dot_all = [0;0;0;0;0;q_dot];
            % update q
            q(:,index) = q(:,index) + q_dot_all *dt;
            q_rcm(:,index) = ConvertToRcm(q(:,index),coupling_matrix);
        end
    end
    
    % transform point clouds boundary box
    for index = 1:3
        frames(:,:,:,index) = robot_kinematics.CalculateFK(q_rcm(:,index),transformation_base(:,:,index));
        for i = 7:10
            rotation = frames(1:3,1:3,i,index);
            translation = frames(1:3,4,i,index);
            for index_boundary = 1 : length(point_boundary_arm{i})
                point_boundary_transformed{i,index}(index_boundary,:) = (rotation * point_boundary_arm{i}(index_boundary,:)' + translation)'; % transformed point clouds stored separately
            end
        end
    end
    
    % detect collision
    [collision12,colliding_arm_index12] = FindCollisionTwoRobot (point_boundary_transformed(:,1),point_boundary_transformed(:,2),index_car:index_pitch_c);
    [collision13,colliding_arm_index13] = FindCollisionTwoRobot (point_boundary_transformed(:,1),point_boundary_transformed(:,3),index_car:index_pitch_c);
    [collision23,colliding_arm_index23] = FindCollisionTwoRobot (point_boundary_transformed(:,2),point_boundary_transformed(:,3),index_car:index_pitch_c);
    
    % change arm color for colliding links
    arm_color1 = GetRobotColor(robot_kinematics);
    arm_color2 = GetRobotColor(robot_kinematics);
    arm_color3 = GetRobotColor(robot_kinematics);
    [arm_color1,arm_color2] = ChangeLinkColor(arm_color1,arm_color2,collision12,colliding_arm_index12);
    [arm_color1,arm_color3] = ChangeLinkColor(arm_color1,arm_color3,collision13,colliding_arm_index13);
    [arm_color2,arm_color3] = ChangeLinkColor(arm_color2,arm_color3,collision23,colliding_arm_index23);
    
    % save joint values
    if do_save
        num_q_store = num_q_store + 1;
        q_store{1}(:,num_q_store) = q(:,1);
        q_store{2}(:,num_q_store) = q(:,2);
        q_store{3}(:,num_q_store) = q(:,3);
        if collision12 || collision13 || collision23
            num_collision = num_collision + 1;
            q_store_collision(:,:,num_collision) = q;
            collision_store(:,num_collision) = [collision12;collision13;collision23];
        end
    end
    
    % plot robot
    if do_plot
        cla
        DrawCoordinateSystem([0.02 0.02 0.02],rotation_camera,translation_camera,'rgb','c')
        hold on
        DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
        hold on
        DrawRobot(frames(:,:,:,1),vertex_arm_origin,arm_color1)
        hold on
        DrawRobot(frames(:,:,:,2),vertex_arm_origin,arm_color2)
        hold on
        DrawRobot(frames(:,:,:,3),vertex_arm_origin,arm_color3)
        hold on
        
        if collision12 || collision13 || collision23
            title(['Collision12 =' num2str(collision12) ';   ' 'Collision13 =' num2str(collision13) ';   ' 'Collision23 =' num2str(collision23)],'Color','r')
        else
            title(['Collision12 =' num2str(collision12) ';   ' 'Collision13 =' num2str(collision13) ';   ' 'Collision23 =' num2str(collision23)],'Color','b')
        end
        
        vertex_hernia_patient_body_transformed = transformSTL(vertex_hernia_patient_body,rotation_hernia_patient,translation_hernia_patient);
        rgba = [0 0 1 0.1];
        PlotStl(vertex_hernia_patient_body_transformed,rgba);
        hold on
        axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
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
    % calculate the time separation
    dt_sep = (time(sample_last_index) - time(sample_first_index))/(sample_last_index - sample_first_index)*sample_rate;
    time_sep = 0 : dt_sep : (sample_last_index - sample_first_index)/sample_rate * dt_sep;
    % calculate qd
    for i = 1 : 3
        for j = 1 : 11
            qd_store{i}(j,:) = diff(q_store{i}(j,:))/dt_sep;
        end
    end
    % calculate qdd
    for i = 1 : 3
        for j = 1 : 11
            qdd_store{i}(j,:) = diff(qd_store{i}(j,:))/dt_sep;
        end
    end
    figure(2)
    plot(time_sep,q_store{1}(6:11,:));
    ylabel('q(rad)');
    xlabel('t(s)');
    legend('roll','pitch','trans','rotate','wr','dis_wr')
    title('joint angle')
    figure(3)
    plot(time_sep(1:length(time_sep)-1),qd_store{1}(6:11,:));
    ylabel('qd(rad/s)');
    xlabel('t(s)');
    legend('roll','pitch','trans','rotate','wr','dis_wr')
    title('joint velocity')
    figure(4)
    plot(time_sep(1:length(time_sep)-2),qdd_store{1}(6:11,:));
    ylabel('qdd(rad/s2)');
    xlabel('t(s)');
    legend('roll','pitch','trans','rotate','wr','dis_wr')
    title('joint acceleration')
end