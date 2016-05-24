%% Collision detection simulation
% This simulation reads the logs of hernia on simulation data and detect
% the collision between arms. This analysis first import the the joint
% values for hernia setup. The joint values for hernia setup is calculated
% from "AnalysisHerniaProcedureSetup.m" and saved in "q_init_setup_hernia.mat".
% The init condition for trocar position, target position and table adapter
% position are from "InitHerniaSetup.m". The log file is input from
% "InitLogToolPath.m" and saved in "hernia_tool_path.mat". The vertex data
% are stls models. "vertex_bed.mat" and "vertex_bed_adapter.mat" have table
% and table adapter information in them.
% set is_artificial = 0 if want to load the real tool path
% set is_artificial = 1 if want to create artificial path that the arm will
% collide
%%
clc
clear all
close all
% robot paramter
load('urdf_info_1.0.mat');
load('arm_version_1.0.mat');
% vertex
load('vertex_arm_origin_1.0.mat');
load('point_boundary_arm_1.0.mat');
load('vertex_patient_body.mat');
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
% robot setup
load('q_init_setup_hernia_crossing_arm.mat');
% tool path
load('egg_sponge_user03_tool_path_raw_with_angle.mat');
% others
load('index_joints.mat');
load('coupling_matrix.mat');

% procedure setup
InitHerniaSetup;
% iterative inverse kinematics parameters
InitIKParameters;
% set simulation
is_artificial = 0; % set to 1 to create artificial colliding simulation
% figure parameters
figure_handle = figure(1);
set(figure_handle,'Position',[600 10 750 750])
hold on
axis equal
view(3)
view(0,65)
% camzoom(5)
trocar_dis = 0.0377 + 0.05;
index_robot = 0;
for index_bed_adapter = selected_bed_adapter;
    if index_bed_adapter ~=0 && index_bed_adapter <= 8
        index_robot = index_robot + 1;
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
        transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    end
end
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
% initialze all the arms
for index_robot = 1:3
    q(:,index_robot) = q_init_setup(:,index_robot);
    robot_arms{index_robot}.transformation_base_ = transformation_base(:,:,index_robot);
    robot_arms{index_robot}.CalculateFK(q(:,index_robot));
end

% setup bed adapter
% q_bed_adapter = [0 -0.02 0;-40*pi/180 -0.18 0;0 -0.02 0]';
p2_tip = robot_arms{2}.frames_(1:3,4,18) + robot_arms{2}.frames_(1:3,3,18) * trocar_dis;
p3_tip = robot_arms{3}.frames_(1:3,4,18) + robot_arms{3}.frames_(1:3,3,18) * trocar_dis;
force_dir = (p3_tip - p2_tip)/norm(p3_tip-p2_tip);
force_tip = 0.90718474 * 9.81;
force2 = force_tip * force_dir;
force3 = -force_tip * force_dir;
jacobian_2 = robot_arms{2}.CalculateJacobianGeneral(robot_arms{2}.frames_(1:3,3,14),10);
jacobian_3 = robot_arms{3}.CalculateJacobianGeneral(robot_arms{3}.frames_(1:3,3,14),10);
tau_2 = jacobian_2(1:3,:)' * force2;
tau_3 = jacobian_3(1:3,:)' * force3;
jacobian_rcm2 = robot_arms{2}.CalculateJacobianGeneral(p2_tip,10);
jacobian_rcm3 = robot_arms{3}.CalculateJacobianGeneral(p3_tip,10);
force2tip = pinv(jacobian_rcm2(1:3,:)') * tau_2;
force3tip = pinv(jacobian_rcm3(1:3,:)') * tau_3;

norm_force2 = norm(force2tip)
norm_force3 = norm(force3tip)

arm_color1 = robot_arms{1}.color_;
arm_color2 = robot_arms{2}.color_;
arm_color3 = robot_arms{3}.color_;


% plot robot

cla
DrawCoordinateSystem([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
hold on
robot_arms{1}.DrawRobot(vertex_arm_origin)
hold on
robot_arms{2}.DrawRobot(vertex_arm_origin)
hold on
robot_arms{3}.DrawRobot(vertex_arm_origin)
hold on
frames_bed_adapter1 = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,selected_bed_adapter(1)));
frames_bed_adapter2 = CalculateBedAdapterFK(q_bed_adapter(:,2),frames_bed_adapter_base(:,:,selected_bed_adapter(2)));
frames_bed_adapter3 = CalculateBedAdapterFK(q_bed_adapter(:,3),frames_bed_adapter_base(:,:,selected_bed_adapter(3)));
DrawBed(vertex_bed,[0.0 0.7 0.0 1])
hold on
DrawBedAdapter(frames_bed_adapter1,vertex_bed_adapter,[1 0 0 1])
hold on
DrawBedAdapter(frames_bed_adapter2,vertex_bed_adapter,[1 0 0 1])
hold on
DrawBedAdapter(frames_bed_adapter3,vertex_bed_adapter,[1 0 0 1])
hold on

vertex_patient_body_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
rgba = [0 0 1 0.1];
PlotStl(vertex_patient_body_transformed,rgba);
hold on

plot3(p2_tip(1),p2_tip(2),p2_tip(3),'Marker','o','MarkerSize',30,'MarkerFaceColor','r')
plot3(p3_tip(1),p3_tip(2),p3_tip(3),'Marker','o','MarkerSize',30,'MarkerFaceColor','r')

mArrow3(robot_arms{2}.frames_(1:3,4,index_eef), robot_arms{2}.frames_(1:3,4,index_eef) + force2/50,'color','r','stemWidth',0.001,'tipWidth',0.004);
mArrow3(robot_arms{3}.frames_(1:3,4,index_eef), robot_arms{3}.frames_(1:3,4,index_eef) + force3/50,'color','r','stemWidth',0.001,'tipWidth',0.004);
axis([-1.2 1 -0.8 0.8 -0.2 2.8])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
drawnow;


