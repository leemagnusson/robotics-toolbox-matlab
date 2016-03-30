%% Resolved rates single arm
% This code runs resolved rates for single arm with desired task space
% input.
% by Haoran Yu 3/16/2016
%%
% init
clc
clear all
close all
load('URDF_info.mat')
load('VertexData_origin.mat')
base_T = eye(4);

% init resolved rates
q_init = [0,0,0,0,0,0,0,0,0,pi/3,pi/3]';
dt = 0.0025;
q = q_init;
q_rcm = convert2rcm(q);
Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
p_eef = Arm_Kinematics_init(14).Tran_matrix(1:3,4);
R_eef = Arm_Kinematics_init(14).Tran_matrix(1:3,1:3);
p_t = p_eef + [0.05;0.05;0.05]; % target position
R_t = R_eef * RotationMatrix_rad(pi/3,[1;0;0]) * RotationMatrix_rad(pi/4,[0;1;0]); % target orientation
ncycle_t = 5;
ncycle_r = 2;
v_max = 0.25;
omega_max = 10.5;
p_eps = 0.001;
theta_eps = 1 * pi/180;

%% resolved rates

figure(1)
hold on
view(49,16)
camzoom(5)
axis equal
p_err = [100;100;100];
theta_err = 100;
index = 1;
movie_index = 1;
while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
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
    [J_rcm,J_car,J_all] = calc_Jacobian_all(Arm_Kinematics1);
    % Calculate Jacobian and q_dot
    %     J = J_all;
    J = J_rcm;
    q_dot = pinv(J)*t_eef;
    q_dot_store(:,index) = q_dot;
    index = index + 1;
    q_dot_all = [0;0;0;0;0;q_dot];
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
    
    %     plot end effector
    
    %     plot3(p_eef(1),p_eef(2),p_eef(3),'Marker','*','MarkerSize',20)
    %     hold on
    draw_coordinate_system([0.1 0.1 0.1],R_t,p_t,'rgb','t')
    hold on
    axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
    F(movie_index) = getframe;
        movie_index = movie_index + 1;
end