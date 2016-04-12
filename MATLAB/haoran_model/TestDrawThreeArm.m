%% Test the robot
% Plot the robot at different joint input
% can draw coordinate system
% by Haoran Yu 3/16/2016
%%
clc
clear all
close all
load('urdf_info.mat');
load('vertex_arm_origin.mat');
load('vertex_hernia_patient_body.mat');
load('arm_version_1.0.mat')
load('q_init_setup_hernia.mat');
load('point_boundary_arm.mat');
load('index_joints.mat');
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
load('coupling_matrix.mat');
InitHerniaSetup;
arm_color = GetRobotColor(robot_kinematics);
figure(1)
hold on
view(62,28)
axis equal
DrawBed(vertex_bed,[0.0 0.7 0.0 1])
hold on
DrawCoordinateSystem([0.1 0.1 0.1],transformation_bed_base(1:3,1:3),transformation_bed_base(1:3,4),'rgb','b')
hold on
num_bed_adapter = 0;
q_bed_adapter = [0 -0.02 0;-40*pi/180 -0.18 0;0 -0.02 0]';
for index_bed_adapter = [2 1 5]
    num_bed_adapter = num_bed_adapter + 1;
    frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,num_bed_adapter),frames_bed_adapter_base(:,:,index_bed_adapter));
    DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1])
    hold on
    q = q_init_setup(:,num_bed_adapter);
    q_rcm = ConvertToRcm(q,coupling_matrix);
    frames = robot_kinematics.CalculateFK(q_rcm,frames_bed_adapter(:,:,end));
    DrawRobot(frames,vertex_arm_origin,arm_color)
    hold on
end
% Draw hernia
vertex_hernia_patient_transformed = transformSTL(vertex_hernia_patient_body,rotation_hernia_patient,translation_hernia_patient);
patient_color = [0 0 1 0.3];
PlotStl(vertex_hernia_patient_transformed,patient_color);
hold on
% axis([-5 5 -5 5 -5 5])
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
drawnow;