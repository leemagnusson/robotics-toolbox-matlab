%% Resolved rates single arm
% This code runs resolved rates for single arm with desired task space
% input.
%%
% init
clc
clear all
close all
URDF_file= 'V1_Arm_URDF.URDF';
load('VertexData_origin.mat')
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;
base_T = eye(4);

% init resolved rates
q_init = [0;0;0;0;0;0;0;0;0;0;0];
% v_eef = [0.0; 0.6; 0.0];
dt = 0.01;
q = q_init;
q_rcm = convert2rcm(q);
p_t = [0;-0.48;0.15];
R_t = eye(3);
ncycle_t = 5;
ncycle_r = 2;
v_max = 0.5;
omega_max = 2*360 * pi/180;
p_eps = 0.001;
theta_eps = 0.3 * pi/180;

%% resolved rates

figure(1)
hold on
view(49,16)
axis equal
p_err = [100;100;100];
theta_err = 100;
while((norm(p_err) > p_eps) || (abs(theta_err) > 1.5 * theta_eps))
    cla
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
    p_eef = Arm_Kinematics1(14).Tran_matrix(1:3,4);
    R_eef = Arm_Kinematics1(14).Tran_matrix(1:3,1:3);
    p_err = p_t - p_eef;
    if norm(p_err) < p_eps
        v_eef = zeros(3,1);
    elseif norm(p_err) < ncycle_t * v_max * dt
        v_eef = p_err/(ncycle_t*dt);
    else
        v_eef = v_max * p_err/norm(p_err);
    end
    
    R_err = R_t * R_eef';
    theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2)
    vect_err = 1/2*[(R_err(3,2)-R_err(2,3));(R_err(1,3)-R_err(3,1));(R_err(2,1)-R_err(1,2))];
    
    if abs(theta_err) < theta_eps
        omega_eef = zeros(3,1);
    elseif abs(theta_err) < ncycle_r * omega_max * dt
        omega_eef = theta_err * vect_err/(ncycle_r*dt);
    else
        omega_eef = omega_max * vect_err;
    end
    t_eef = [v_eef;omega_eef];
%     t_eef = [0;0.5;0;0;0;0];
    [J_rcm,J_car,J_car_6DoF,J_all] = calc_Jacobian_all(Arm_Kinematics1);
    % Calculate Jacobian and q_dot
    J = J_rcm;
    q_dot = pinv(J) * t_eef;
    q_dot_all = [0;0;0;0;0;q_dot];
    %     J=J_all(1:3,:);
    %     q_dot = pinv(J) * eef_v;
    %     q_dot_all = [q_dot];
    
    % update q
    q = q + q_dot_all *dt;
    q_rcm = convert2rcm(q);
    for i = 1:18
        R = Arm_Kinematics1(i).Tran_matrix(1:3,1:3);
        d = Arm_Kinematics1(i).Tran_matrix(1:3,4);
        % transform vertexdata
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran(:,i) = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics1(i).color;
            plotSTL(VertexData_tran(:,i),rgba)
            hold on
        end
        if ismember(i,[1 14])
            draw_coordinate_system([0.1 0.1 0.1],R,d,'rgb',num2str(i))
            hold on
        end
    end
    
    % plot end effector
    
    %     plot3(p_eef(1),p_eef(2),p_eef(3),'Marker','*','MarkerSize',20)
    %     hold on
    draw_coordinate_system([0.1 0.1 0.1],R_t,p_t,'rgb','t')
    hold on
    axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
    drawnow;
end