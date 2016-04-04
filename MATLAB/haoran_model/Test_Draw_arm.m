%% Test the robot
% Plot the robot at different joint input
% can draw coordinate system
% by Haoran Yu 3/16/2016
%%
clc
clear all
close all
load('Arm_version_1.0.mat')
Arm_color = get_arm_color(Arm_class);
load('VertexData_origin.mat')
base_T=eye(4);
figure(1)
hold on
view(62,28)
axis equal
for j=1:1
    cla
    q=[pi/100*j,0,0,0,0,0,0,0,0,0,0]';
    q_rcm = convert2rcm(q);
    Frames = Arm_class.calc_FK(q_rcm,base_T);
    Draw_Robot_Arm(Frames,VertexData_origin,Arm_color,[10 11 12],0.2);
    axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
end