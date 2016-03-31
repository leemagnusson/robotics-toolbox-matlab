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
q_init = [0,0,0,0,0,0,0,0,0,0,0]';
q_rcm = convert2rcm(q_init);
Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
dt = 0.0025;

ncycle_t = 5;
ncycle_r = 2;
v_max = 0.75;
omega_max = 10.5;
p_eps = 0.001;
theta_eps = 1 * pi/180;

jnt_limits = [Arm_Kinematics1(2).jnt_limit Arm_Kinematics1(3).jnt_limit Arm_Kinematics1(4).jnt_limit Arm_Kinematics1(5).jnt_limit Arm_Kinematics1(6).jnt_limit Arm_Kinematics1(7).jnt_limit Arm_Kinematics1(8).jnt_limit Arm_Kinematics1(11).jnt_limit Arm_Kinematics1(12).jnt_limit Arm_Kinematics1(13).jnt_limit Arm_Kinematics1(14).jnt_limit];
q_mid = 1/2 * (jnt_limits(1,:) + jnt_limits(2,:));
%% resolved rates
figure(1)
hold on
view(46,23)

axis equal
p_err = [100;100;100];
theta_err = 100;
index = 1;
movie_index = 1;
for theta = -pi/2 : pi/180 : pi/2
    for psi = -pi : pi/180 : pi
%         vel_dir = [cos(theta)*cos(psi);cos(theta)*sin(psi);sin(theta)];
vel_dir = [1;0;0];
        reached = 0;
        q = q_init;
        q_rcm = convert2rcm(q);
        Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
        while(reached == 0)
            p_wr = Arm_Kinematics1(13).Tran_matrix(1:3,4);
            v_eef = v_max * vel_dir;
            t_eef = [0;0;0;v_eef];
            [J_rcm,J_car,J_all] = calc_Jacobian_all(Arm_Kinematics1);
            J = [J_car(1:3,:) zeros(3,4);J_all(1:3,1:9)];
            q_dot = pinv(J)*t_eef + (eye(9) - pinv(J) * J) * (q(1:9) - q_init(1:9));
            q_dot_all = [q_dot;0;0];
            % update q
            q = q + q_dot_all *dt;
%             q'
            q_rcm = convert2rcm(q);
            Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
            p_rcm = Arm_Kinematics1(18).Tran_matrix(1:3,4);
            cla
            Draw_Robot_Arm(Arm_Kinematics1,VertexData_origin)
            hold on
            axis([-1 1 -1 0.5 -0.2 1.2])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
        end
    end
end
save('eef_workspace2.mat','eef_store');

% load('eef_workspace2.mat');
% boundary_eef = boundary(eef_store',0.9);
% light('Position',[1 3 2]);
% light('Position',[-3 -1 -3]);
% trisurf(boundary_eef,eef_store(1,:)',eef_store(2,:)',eef_store(3,:)','Facecolor','red','FaceAlpha',0.3,'EdgeColor','none');
% axis([-1 1 -1 0.5 -0.2 1.2])
% grid on
%
% drawnow;