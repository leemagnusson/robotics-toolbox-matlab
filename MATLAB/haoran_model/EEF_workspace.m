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
view(46,23)
axis equal
q=[0,0,0,0,0,0,0,0,0,0,0]';
q_rcm = convert2rcm(q);
Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
Draw_Robot_Arm(Arm_Kinematics1,VertexData_origin,[1]);
hold on
jnt_limits = [[-pi;pi] Arm_Kinematics1(8).jnt_limit Arm_Kinematics1(11).jnt_limit [-pi;pi] Arm_Kinematics1(13).jnt_limit Arm_Kinematics1(14).jnt_limit];
num_data = 1;
sample_step = 10*pi/180;
% for q6 = jnt_limits(1,1):(sample_step):jnt_limits(2,1)
%     for q7 = jnt_limits(1,2):sample_step:jnt_limits(2,2)
%         for q8 = jnt_limits(1,3):0.01:jnt_limits(2,3)
% %             for q9 = jnt_limits(1,4):(6*sample_step):jnt_limits(2,4)
% %                 for q10 = jnt_limits(1,5):(6*sample_step):jnt_limits(2,5)
%                     q=[0,0,0,0,0,q6,q7,q8,0,0,0]';
%                     q_rcm = convert2rcm(q);
%                     Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
%                     eef_store(:,num_data) = Arm_Kinematics1(14).Tran_matrix(1:3,4);
%                     num_data = num_data + 1;
% %                 end
% %             end
%         end
%     end
% end
% save('eef_workspace2.mat','eef_store');

load('eef_workspace2.mat');
% num_filtered = 1;
% for i = 1 : length(eef_store)
%     if eef_store(3,i)>0 && eef_store(2,i)<0
%         eef_filtered(:,num_filtered) = eef_store(:,i);
%         num_filtered = num_filtered + 1;
%     end
% end
% boundary_eef = boundary(eef_filtered');
boundary_eef = boundary(eef_store',0.9);
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
trisurf(boundary_eef,eef_store(1,:)',eef_store(2,:)',eef_store(3,:)','Facecolor','red','FaceAlpha',0.3,'EdgeColor','none');
axis([-1 1 -1 0.5 -0.2 1.2])
grid on

drawnow;