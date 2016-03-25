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
load('q_init_setup_hernia_optimized4.mat')
% init resolved rates

dt = 0.01;
ncycle_t = 5;
ncycle_r = 2;
v_max = 0.25;
omega_max = 10.5;
p_eps = 0.001;
theta_eps = 1 * pi/180;
figure(1)
hold on
view(3)
view(-2,56)
% camzoom(5)
axis equal

for index = 1:3
    
    q = q_init_setup(:,index);
    q_rcm = convert2rcm(q);
    Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm,base_T_setup(:,:,index));
    p_rcm = Arm_Kinematics_init(18).Tran_matrix(1:3,4);
    R_rcm = Arm_Kinematics_init(7).Tran_matrix(1:3,1:3);
    p_t = Trocar(:,index); % target position
    R_t = R_Trocar(:,:,index); % target orientation
    
    %% resolved rates
    
    p_err = p_t - p_rcm;
    R_err = R_t * R_rcm';
    theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
    while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
        cla
        Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T_setup(:,:,index));
        p_rcm = Arm_Kinematics1(18).Tran_matrix(1:3,4);
        R_rcm = Arm_Kinematics1(7).Tran_matrix(1:3,1:3);
        p_err = p_t - p_rcm;
        if norm(p_err) < p_eps
            v_eef = zeros(3,1);
        elseif norm(p_err) < ncycle_t * v_max * dt
            v_eef = p_err/(ncycle_t*dt);
        else
            v_eef = v_max * p_err/norm(p_err);
        end
        
        R_err = R_t * R_rcm';
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
        [J_6DoF] = calc_Jacobian_6DoF_rcm(Arm_Kinematics1);
        % Calculate Jacobian and q_dot
        %     J = J_all;
        J = J_6DoF;
        q_dot = pinv(J)*t_eef;
        q_dot_all = [q_dot;0;0;0;0;0];
        % update q
        q = q + q_dot_all *dt;
        q_rcm = convert2rcm(q);
        Draw_Robot_Arm(Arm_Kinematics1,VertexData_origin)
        
        draw_coordinate_system([0.1 0.1 0.1],R_t,p_t,'rgb','t')
        hold on
        draw_coordinate_system([0.1 0.1 0.1],Arm_Kinematics1(7).Tran_matrix(1:3,1:3),Arm_Kinematics1(18).Tran_matrix(1:3,4),'rgb','rcm')
        hold on
        draw_coordinate_system([0.1 0.1 0.1],Arm_Kinematics1(7).Tran_matrix(1:3,1:3),Arm_Kinematics1(7).Tran_matrix(1:3,4),'rgb','roll')
        hold on
        %     axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
        drawnow;
    end
    
    
    q_rcm = convert2rcm(q);
    Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm,base_T_setup(:,:,index));
    p_eef = Arm_Kinematics_init(12).Tran_matrix(1:3,4);
    p_t = 0.9 * Hernia + 0.1 * Trocar(:,index); % target position
    R_t = R_Trocar(:,:,index); % target orientation
    
    p_err = p_t - p_eef;
    
    while((norm(p_err) > p_eps))
        cla
        Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T_setup(:,:,index));
        p_eef = Arm_Kinematics1(12).Tran_matrix(1:3,4);
        p_err = p_t - p_eef;
        if norm(p_err) < p_eps
            v_eef = zeros(3,1);
        elseif norm(p_err) < ncycle_t * v_max * dt
            v_eef = p_err/(ncycle_t*dt);
        else
            v_eef = v_max * p_err/norm(p_err);
        end
        
        [J_rcm,J_car,J_all] = calc_Jacobian_all(Arm_Kinematics1);
        % Calculate Jacobian and q_dot
        %     J = J_all;
        J = J_rcm(1:3,1:3);
        q_dot = pinv(J)*v_eef;
        q_dot_all = [0;0;0;0;0;q_dot;0;0;0];
        % update q
        q = q + q_dot_all *dt;
        q_rcm = convert2rcm(q);
        Draw_Robot_Arm(Arm_Kinematics1,VertexData_origin)
        
        draw_coordinate_system([0.1 0.1 0.1],R_t,p_t,'rgb','t')
        hold on
        
        drawnow;
    end
    q_init_setup(:,index) = wrap2pi(q);
end
% save('q_init_setup.mat','q_init_setup');