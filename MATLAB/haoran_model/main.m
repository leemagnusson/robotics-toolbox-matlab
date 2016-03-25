%% Test the robot
% Plot the robot at different joint input
% can draw coordinate system
% by Haoran Yu 3/16/2016
%%
clc
clear all
close all
load('URDF_info.mat')
load('VertexData_origin.mat')
base_T=eye(4);

figure(1)
hold on
view(62,28)
axis equal
for j=1:1
%     cla
    q=[-pi/4,pi/4,0,0,pi/4,pi,-pi/4,0,0,0,0]';
    q_rcm = convert2rcm(q);
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
    Draw_Robot_Arm(Arm_Kinematics1,VertexData_origin,[1 2 3]);
    T_eef = Arm_Kinematics1(14).Tran_matrix;
    eef = T_eef(1:3,4);
    Arm_Kinematics1(18).Tran_matrix(1:3,4)
    norm(Arm_Kinematics1(18).Tran_matrix(1:3,4)-Arm_Kinematics1(14).Tran_matrix(1:3,4));
%     plot3(eef(1),eef(2),eef(3),'Marker','*','MarkerSize',20)
%     hold on
    axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
    drawnow;
end
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);