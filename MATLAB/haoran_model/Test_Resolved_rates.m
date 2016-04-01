%% Resolved rates single arm
% This code runs resolved rates for single arm with desired task space
% input.
% by Haoran Yu 3/16/2016
%%
% init
clc
clear all
close all
load('/data_store/URDF_info.mat')
load('/data_store/VertexData_origin.mat')
load('/data_store/Arm_version_1.0.mat')

% init resolved rates
base_T = eye(4);
q_init = [0,0,0,0,0,0,0,0,0,pi/3,pi/3]';
init_IK_parameters;
q = q_init;
q_rcm = convert2rcm(q);
Frames = Arm_class.calc_FK(q_rcm,base_T);
p_eef = Frames(1:3,4,14); % eef position
R_eef = Frames(1:3,1:3,14); % eef orientation
p_t = p_eef + [0.05;0.05;0.05]; % target position
R_t = R_eef * rotr([1;0;0],pi/3) * rotr([0;1;0],pi/4); % target orientation
p_err = p_t - p_eef;
R_err = R_t * R_eef';
theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
Arm_color = get_arm_color(Arm_class);
%% resolved rates
figure(1)
hold on
view(49,16)
camzoom(5)
axis equal
movie_index = 1;
while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
    cla
    Frames_cur = Arm_class.calc_FK(q_rcm,base_T);
    p_eef = Frames_cur(1:3,4,14);
    R_eef = Frames_cur(1:3,1:3,14);
    % compute twist
    [t_eef,p_err,theta_err] = compute_twist(p_t,p_eef,R_t,R_eef);
    % get Jacobian
    [J_rcm,J_car,J_all] = calc_Jacobian_all(Frames_cur);
    J = J_rcm;
    q_dot = pinv(J)*t_eef;
    q_dot_all = [0;0;0;0;0;q_dot];
    % update q
    q = q + q_dot_all *dt;
    q_rcm = convert2rcm(q);
    Draw_Robot_Arm(Frames_cur,VertexData_origin,Arm_color,[14],0.1)
    hold on
    draw_coordinate_system([0.1 0.1 0.1],R_t,p_t,'rgb','t')
    hold on
    axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
    F(movie_index) = getframe;
    movie_index = movie_index + 1;
end