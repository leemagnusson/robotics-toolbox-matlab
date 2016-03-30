%% Resolved rates single arm
% This code runs resolved rates for single arm with desired task space
% input.
% by Haoran Yu 3/16/2016
%%
% init
clc
clear all
close all
ARM_setup_hernia
load('URDF_info.mat')
load('VertexData_origin.mat')
load('VertexData_Hernia_Body.mat');
% load('q_init_setup_hernia_optimized4.mat')
% init resolved rates

dt = 0.01;
ncycle_t = 5;
ncycle_r = 2;
v_max = 0.75;
omega_max = 10.5;
p_eps = 0.001;
theta_eps = 1 * pi/180;
fig_handle = figure(1);
set(fig_handle,'Position',[600 10 750 750])
hold on
view(3)
view(0,90)
% camzoom(1.2)
% camzoom(5)
axis equal
movie_index = 1;

%     q = q_init_setup(:,index);
for index =1:3
    q(:,index) = q_set(:,index);
    q_rcm(:,index) = convert2rcm(q(:,index));
    Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm(:,index),base_T_setup(:,:,index));
    p_rcm(:,index) = Arm_Kinematics_init(18).Tran_matrix(1:3,4);
    R_rcm(:,:,index) = Arm_Kinematics_init(7).Tran_matrix(1:3,1:3);
    p_t(:,index) = Trocar(:,index); % target position
    R_t(:,:,index) = R_Trocar(:,:,index); % target orientation
end
converged = zeros(3,1);
for index = 1:3
    %% resolved rates
    
    p_err = p_t(:,index) - p_rcm(:,index);
    R_err = R_t(:,:,index) * R_rcm(:,:,index)';
    theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
    while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
        cla
        Arm_Kinematics_cur = Arm_Kinematics(link_input,joint_input,q_rcm(:,index),base_T_setup(:,:,index));
        p_rcm(:,index) = Arm_Kinematics_cur(18).Tran_matrix(1:3,4);
        R_rcm(:,:,index) = Arm_Kinematics_cur(7).Tran_matrix(1:3,1:3);
        p_err = p_t(:,index) - p_rcm(:,index);
        if norm(p_err) < p_eps
            v_eef = zeros(3,1);
        elseif norm(p_err) < ncycle_t * v_max * dt
            v_eef = p_err/(ncycle_t*dt);
        else
            v_eef = v_max * p_err/norm(p_err);
        end
        
        R_err = R_t(:,:,index) * R_rcm(:,:,index)';
        theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
        vect_err = 1/(2*sin(theta_err))*[(R_err(3,2)-R_err(2,3));(R_err(1,3)-R_err(3,1));(R_err(2,1)-R_err(1,2))];
        
        if abs(theta_err) < theta_eps
            omega_eef = zeros(3,1);
        elseif abs(theta_err) < ncycle_r * omega_max * dt
            omega_eef = theta_err * vect_err/(ncycle_r*dt);
        else
            omega_eef = omega_max * vect_err;
        end
        t_eef = [v_eef;omega_eef];
        [J_6DoF] = calc_Jacobian_6DoF_rcm(Arm_Kinematics_cur);
        % Calculate Jacobian and q_dot
        %     J = J_all;
        J = J_6DoF;
        q_dot = pinv(J)*t_eef;
        q_dot_all = [q_dot;0;0;0;0;0];
        % update q
        q(:,index) = q(:,index) + q_dot_all *dt;
        q_rcm(:,index) = convert2rcm(q(:,index));
        Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm(:,1),base_T_setup(:,:,1));
        Arm_Kinematics2 = Arm_Kinematics(link_input,joint_input,q_rcm(:,2),base_T_setup(:,:,2));
        Arm_Kinematics3 = Arm_Kinematics(link_input,joint_input,q_rcm(:,3),base_T_setup(:,:,3));
        Draw_Robot_Arm_no_tool(Arm_Kinematics1,VertexData_origin)
        hold on
        Draw_Robot_Arm_no_tool(Arm_Kinematics2,VertexData_origin)
        hold on
        Draw_Robot_Arm_no_tool(Arm_Kinematics3,VertexData_origin)
        hold on
        draw_coordinate_system([0.02 0.02 0.02],R_t(:,:,index),p_t(:,index),'rgb')
        hold on
        draw_coordinate_system([0.02 0.02 0.02],Arm_Kinematics_cur(7).Tran_matrix(1:3,1:3),Arm_Kinematics_cur(18).Tran_matrix(1:3,4),'rgb')
        hold on
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
    q_rcm(:,index) = convert2rcm(q(:,index));
    Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm(:,index),base_T_setup(:,:,index));
    p_eef(:,index) = Arm_Kinematics_init(12).Tran_matrix(1:3,4);
    if index==2
        p_t(:,index) = p_Camera;
    else
        p_t(:,index) = 0.9 * Hernia + 0.1 * Trocar(:,index); % target position
    end
    
    R_t(:,:,index) = R_Trocar(:,:,index); % target orientation
    
    p_err = p_t(:,index) - p_eef(:,index);
    
    while((norm(p_err) > p_eps))
        cla
        Arm_Kinematics_cur = Arm_Kinematics(link_input,joint_input,q_rcm(:,index),base_T_setup(:,:,index));
        p_eef(:,index) = Arm_Kinematics_cur(12).Tran_matrix(1:3,4);
        p_err = p_t(:,index) - p_eef(:,index);
        if norm(p_err) < p_eps
            v_eef = zeros(3,1);
        elseif norm(p_err) < ncycle_t * v_max * dt
            v_eef = p_err/(ncycle_t*dt);
        else
            v_eef = v_max * p_err/norm(p_err);
        end
        
        [J_rcm,J_car,J_all] = calc_Jacobian_all(Arm_Kinematics_cur);
        % Calculate Jacobian and q_dot
        %     J = J_all;
        J = J_rcm(1:3,1:3);
        q_dot = pinv(J)*v_eef;
        q_dot_all = [0;0;0;0;0;q_dot;0;0;0];
        % update q
        q(:,index) = q(:,index) + q_dot_all *dt;
        q_rcm(:,index) = convert2rcm(q(:,index));
        
        Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm(:,1),base_T_setup(:,:,1));
        Arm_Kinematics2 = Arm_Kinematics(link_input,joint_input,q_rcm(:,2),base_T_setup(:,:,2));
        Arm_Kinematics3 = Arm_Kinematics(link_input,joint_input,q_rcm(:,3),base_T_setup(:,:,3));
        Draw_Robot_Arm(Arm_Kinematics1,VertexData_origin)
        hold on
        Draw_Robot_Arm(Arm_Kinematics2,VertexData_origin)
        hold on
        Draw_Robot_Arm(Arm_Kinematics3,VertexData_origin)
        hold on
        
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
% save('q_init_setup.mat','q_init_setup');