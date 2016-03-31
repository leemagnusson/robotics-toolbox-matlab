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
v_eef = [0.0; 0.6; 0.0];
dt = 0.01;
q = q_init;
q_rcm = convert2rcm(q);

%% resolved rates

figure(1)
hold on
view(62,28)
axis equal
for j=1:100
%     cla
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
    [J_rcm,J_car,J_car_6DoF,J_all] = calc_Jacobian_all(Arm_Kinematics1);
    % Calculate Jacobian and q_dot
    J = J_rcm(1:3,1:4);
    q_dot = pinv(J) * v_eef;
    q_dot_all = [0;0;0;0;0;q_dot;0;0];
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
    end
    % plot end effector
    eef = Arm_Kinematics1(13).Tran_matrix(1:3,4);
    plot3(eef(1),eef(2),eef(3),'Marker','*','MarkerSize',20)
    hold on
    axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
    drawnow;
end