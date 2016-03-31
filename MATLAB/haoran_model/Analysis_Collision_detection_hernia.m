%% Collision detection simulation
% This simulation reads the logs of hernia on simulation data and detect
% the collision between arms.
% set is_artificial = 0 if want to load the real tool path
% set is_artificial = 1 if want to create artificial path that the arm will
% collide
% by Haoran Yu 3/28/2016
%%
clc
clear all
close all
load('/../data_store/URDF_info.mat');
load('/../data_store/VertexData_origin.mat');
load('/../data_store/VertexData_Hernia_Body.mat');
load('/../data_store/Arm_version_1.0.mat')
% load('/../data_store/q_init_setup.mat');
load('/../data_store/q_init_setup_hernia_best.mat');
load('/../data_store/Tool_path.mat');
load('/../data_store/point_boundary_all.mat');
load('/../data_store/index_joints.mat');
init_Hernia_setup;
% iterative inverse kinematics parameters
init_IK_parameters;
% set simulation 
is_artificial = 0; % set to 1 to create artificial colliding simulation
% figure parameters
fig_handle = figure(1);
set(fig_handle,'Position',[600 10 750 750])
hold on
axis equal
view(3)
view(-2,43)

% set index
movie_index = 1;
number_col = 1;
num_q = 1;
index_start = 1767;
index_end = 2592;
sample_rate = 100; % change the sample rate for simulation
do_plot = 1; % set to 1 if want to plot
do_save = 0; % set to 1 if want to store joint values
do_save_video = 0; % set to 1 if want to save video

% initialze all the arms
q(:,1) = q_init_setup(:,1);
q_rcm(:,1) = convert2rcm(q_init_setup(:,1));
Frames1 = Arm_class.calc_FK(q_rcm(:,1),base_T_setup(:,:,1));
p_eef(:,1) = Frames1(1:3,4,index_eef);
R_eef(:,:,1) = Frames1(1:3,1:3,index_eef);

q(:,2) = q_init_setup(:,2);
q_rcm(:,2) = convert2rcm(q_init_setup(:,2));
Frames2 = Arm_class.calc_FK(q_rcm(:,2),base_T_setup(:,:,2));
p_eef(:,2) = Frames2(1:3,4,index_eef);
R_eef(:,:,2) = Frames2(1:3,1:3,index_eef);

q(:,3) = q_init_setup(:,3);
q_rcm(:,3) = convert2rcm(q_init_setup(:,3));
Frames3 = Arm_class.calc_FK(q_rcm(:,3),base_T_setup(:,:,3));
p_eef(:,3) = Frames3(1:3,4,index_eef);
R_eef(:,:,3) = Frames3(1:3,1:3,index_eef);

for sample_index = index_start : sample_rate : index_end
    % transform the tool path on all the arms into world frame
    Tool_path_left(1:3,4,sample_index) = R_Camera * Tool_path_left(1:3,4,1) + p_Camera;
    Tool_path_left(1:3,1:3,sample_index) = R_Camera * Tool_path_left(1:3,1:3,sample_index);
    if is_artificial
        Tool_path_right(1:3,4,sample_index) = R_Camera * Tool_path_right(1:3,4,1) + p_Camera + [-0.00025*(sample_index-1767);0;0];
    else
        Tool_path_right(1:3,4,sample_index) = R_Camera * Tool_path_right(1:3,4,sample_index) + p_Camera;
    end
    Tool_path_right(1:3,1:3,sample_index) = R_Camera * Tool_path_right(1:3,1:3,sample_index);
    
    % set the target pose for arm 1 and arm 3.
    p_t(:,1) = Tool_path_left(1:3,4,sample_index);
    p_t(:,3) = Tool_path_right(1:3,4,sample_index);
    
    R_t(:,:,1) = Tool_path_left(1:3,1:3,sample_index);
    R_t(:,:,3) = Tool_path_right(1:3,1:3,sample_index);
    
    % move arm 1 and arm 3
    for index = [1 3]
        p_err = p_t(:,index) - p_eef(:,index);
        R_err = R_t(:,:,index) * R_eef(:,:,index)';
        theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
        
        % iterative IK
        while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
            % compute the current frames
            Frames_cur = Arm_class.calc_FK(q_rcm(:,index),base_T_setup(:,:,index));
            % compute eef pose
            p_eef(:,index) = Frames_cur(1:3,4,index_eef);
            R_eef(:,:,index) = Frames_cur(1:3,1:3,index_eef);
            % compute twist
            [t_eef,p_err,theta_err] = compute_twist(p_t(:,index),p_eef(:,index),R_t(:,:,index),R_eef(:,:,index));
            % compute Jacobian
            [J_rcm,J_car,J_all] = calc_Jacobian_all(Frames_cur);
            % compute joint velocity
            q_dot = pinv(J_rcm)*t_eef;
            q_dot_all = [0;0;0;0;0;q_dot];
            % update q
            q(:,index) = q(:,index) + q_dot_all *dt;
            q_rcm(:,index) = convert2rcm(q(:,index));
        end
    end
    
    % transform point clouds boundary box
    for index = 1:3
        Frames(:,:,:,index) = Arm_class.calc_FK(q_rcm(:,index),base_T_setup(:,:,index));
        for i = 7:10
            R = Frames(1:3,1:3,i,index);
            d = Frames(1:3,4,i,index);
            for index_boundary = 1 : length(point_boundary{i})
                point_boundary_tran{i,index}(index_boundary,:) = (R * point_boundary{i}(index_boundary,:)' + d)'; % transformed point clouds stored separately
            end
        end
    end
    
    % detect collision
    [Collision12,collide_index12] = find_collision_two_arm (point_boundary_tran(:,1),point_boundary_tran(:,2),index_car:index_pitch_c);
    [Collision13,collide_index13] = find_collision_two_arm (point_boundary_tran(:,1),point_boundary_tran(:,3),index_car:index_pitch_c);
    [Collision23,collide_index23] = find_collision_two_arm (point_boundary_tran(:,2),point_boundary_tran(:,3),index_car:index_pitch_c);
    
    % change arm color for colliding links
    Arm_color1 = get_arm_color(Arm_class);
    Arm_color2 = get_arm_color(Arm_class);
    Arm_color3 = get_arm_color(Arm_class);
    [Arm_color1,Arm_color2] = Change_link_color(Arm_color1,Arm_color2,Collision12,collide_index12);
    [Arm_color1,Arm_color3] = Change_link_color(Arm_color1,Arm_color3,Collision13,collide_index13);
    [Arm_color2,Arm_color3] = Change_link_color(Arm_color2,Arm_color3,Collision23,collide_index23);
    
    % save joint values
    if do_save
        q_store{1}(:,num_q) = q(:,1);
        q_store{2}(:,num_q) = q(:,2);
        q_store{3}(:,num_q) = q(:,3);
        num_q = num_q + 1;
        if Collision12 || Collision13 || Collision23
            q_store_collision(:,:,number_col) = q;
            Collision_store(:,number_col) = [Collision12;Collision13;Collision23];
            number_col = number_col + 1;
        end
    end
    
    % plot robot
    if do_plot
        cla
        draw_coordinate_system([0.02 0.02 0.02],R_Camera,p_Camera,'rgb','c')
        hold on
        draw_coordinate_system([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
        hold on
        Draw_Robot_Arm(Frames(:,:,:,1),VertexData_origin,Arm_color1)
        hold on
        Draw_Robot_Arm(Frames(:,:,:,2),VertexData_origin,Arm_color2)
        hold on
        Draw_Robot_Arm(Frames(:,:,:,3),VertexData_origin,Arm_color3)
        hold on
        
        if Collision12 || Collision13 || Collision23
            title(['Collision12 =' num2str(Collision12) ';   ' 'Collision13 =' num2str(Collision13) ';   ' 'Collision23 =' num2str(Collision23)],'Color','r')
        else
            title(['Collision12 =' num2str(Collision12) ';   ' 'Collision13 =' num2str(Collision13) ';   ' 'Collision23 =' num2str(Collision23)],'Color','b')
        end
        
        VertexData_hernia_tran = transformSTL(VertexData_Hernia_Body,R_Hernia_Body,Hernia_Body);
        rgba = [0 0 1 0.1];
        plotSTL(VertexData_hernia_tran,rgba);
        hold on
        axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        drawnow;
    end
    if do_save_video
        F(movie_index) = getframe(gcf);
        movie_index = movie_index + 1;
    end
end
sample_first_index = index_start;
sample_last_index = sample_index;
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