%% Resolved rates single arm
% This code runs resolved rates for single arm with desired task space
% input.
% by Haoran Yu 3/16/2016
%%
% init
clc
clear all
close all
init_Hernia_setup
load('URDF_info.mat')
load('VertexData_origin.mat')
load('VertexData_Hernia_Body.mat');
load('Arm_version_1.0.mat')
load('index_joints.mat');
% load('q_init_setup_hernia_optimized4.mat')

% init resolved rates
init_IK_parameters;

% init figure
fig_handle = figure(1);
set(fig_handle,'Position',[600 10 750 750])
hold on
view(3)
view(0,90)
% camzoom(1.2)
% camzoom(5)
axis equal
movie_index = 1;
Arm_color = get_arm_color(Arm_class);

% initialize all the arms
for index =1:3
    q(:,index) = q_set(:,index);
    q_rcm(:,index) = convert2rcm(q(:,index));
    Frames_init = Arm_class.calc_FK(q_rcm(:,index),base_T_setup(:,:,index));
    p_rcm(:,index) = Frames_init(1:3,4,index_rcm);
    R_rcm(:,:,index) = Frames_init(1:3,1:3,index_car);
    p_t(:,index) = Trocar(:,index); % target position
    R_t(:,:,index) = R_Trocar(:,:,index); % target orientation
end

% Use 6Dof cartesian to move the rcm link
converged = zeros(3,1);
for index = 1:3
    %% resolved rates
    p_err = p_t(:,index) - p_rcm(:,index);
    R_err = R_t(:,:,index) * R_rcm(:,:,index)';
    theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
    while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
        cla
        Frames_cur = Arm_class.calc_FK(q_rcm(:,index),base_T_setup(:,:,index));
        p_rcm(:,index) = Frames_cur(1:3,4,index_rcm);
        R_rcm(:,:,index) = Frames_cur(1:3,1:3,index_car);
        [t_rcm,p_err,theta_err] = compute_twist(p_t(:,index),p_rcm(:,index),R_t(:,:,index),R_rcm(:,:,index));
        [J_6DoF] = calc_Jacobian_6DoF_rcm(Frames_cur);
        J = J_6DoF;
        q_dot = pinv(J)*t_rcm;
        q_dot_all = [q_dot;0;0;0;0;0];
        % update q
        q(:,index) = q(:,index) + q_dot_all *dt;
        q_rcm(:,index) = convert2rcm(q(:,index));
        
        % Draw robot
        Frames1 = Arm_class.calc_FK(q_rcm(:,1),base_T_setup(:,:,1));
        Frames2 = Arm_class.calc_FK(q_rcm(:,2),base_T_setup(:,:,2));
        Frames3 = Arm_class.calc_FK(q_rcm(:,3),base_T_setup(:,:,3));
        Draw_Robot_Arm_no_tool(Frames1,VertexData_origin,Arm_color)
        hold on
        Draw_Robot_Arm_no_tool(Frames2,VertexData_origin,Arm_color)
        hold on
        Draw_Robot_Arm_no_tool(Frames3,VertexData_origin,Arm_color)
        hold on
        draw_coordinate_system([0.02 0.02 0.02],R_t(:,:,index),p_t(:,index),'rgb')
        hold on
        draw_coordinate_system([0.02 0.02 0.02],Frames_cur(1:3,1:3,index_car),Frames_cur(1:3,4,index_rcm),'rgb')
        hold on
        % Draw hernia
        VertexData_hernia_tran = transformSTL(VertexData_Hernia_Body,R_Hernia_Body,Hernia_Body);
        rgba = [0 0 1 0.1];
        plotSTL(VertexData_hernia_tran,rgba);
        hold on
        axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        drawnow;
        F(movie_index) = getframe(gcf);
        movie_index = movie_index + 1;
    end
end
for index =1:3
    % set rcm pose and target pose
    q_rcm(:,index) = convert2rcm(q(:,index));
    Frames_init = Arm_class.calc_FK(q_rcm(:,index),base_T_setup(:,:,index));
    p_eef(:,index) = Frames_init(1:3,4,12);
    R_eef(:,:,index) = Frames_init(1:3,1:3,12);
    if index==2
        p_t(:,index) = p_Camera;
    else
        p_t(:,index) = 0.9 * Hernia + 0.1 * Trocar(:,index); % target position
    end
    R_t(:,:,index) = R_eef(:,:,index); % target orientation
    % resolved rates
    p_err = p_t(:,index) - p_eef(:,index);
    while((norm(p_err) > p_eps))
        cla
        Frames_cur = Arm_class.calc_FK(q_rcm(:,index),base_T_setup(:,:,index));
        p_eef(:,index) = Frames_cur(1:3,4,index_tool_rotate);
        [t_eef,p_err,theta_err] = compute_twist(p_t(:,index),p_eef(:,index),R_t(:,:,index),R_eef(:,:,index));
        
        [J_rcm,J_car,J_all] = calc_Jacobian_all(Frames_cur);

        J = J_rcm(1:3,1:3);
        v_eef = t_eef(1:3);
        q_dot = pinv(J)*v_eef;
        q_dot_all = [0;0;0;0;0;q_dot;0;0;0];
        % update q
        q(:,index) = q(:,index) + q_dot_all *dt;
        q_rcm(:,index) = convert2rcm(q(:,index));
        
        % Draw robot
        Frames1 = Arm_class.calc_FK(q_rcm(:,1),base_T_setup(:,:,1));
        Frames2 = Arm_class.calc_FK(q_rcm(:,2),base_T_setup(:,:,2));
        Frames3 = Arm_class.calc_FK(q_rcm(:,3),base_T_setup(:,:,3));
        Draw_Robot_Arm(Frames1,VertexData_origin,Arm_color)
        hold on
        Draw_Robot_Arm(Frames2,VertexData_origin,Arm_color)
        hold on
        Draw_Robot_Arm(Frames3,VertexData_origin,Arm_color)
        hold on
        % Draw hernia
        VertexData_hernia_tran = transformSTL(VertexData_Hernia_Body,R_Hernia_Body,Hernia_Body);
        rgba = [0 0 1 0.1];
        plotSTL(VertexData_hernia_tran,rgba);
        hold on
        
        axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
        light('Position',[1 3 2]);
        light('Position',[-3 -1 -3]);
        drawnow;
        F(movie_index) = getframe(gcf);
        movie_index = movie_index + 1;
    end
    
end
q_init_setup(:,index) = wrap2pi(q);
% save('/data_store/q_init_setup.mat','q_init_setup');