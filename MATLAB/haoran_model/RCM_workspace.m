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
% jnt_limits = [Arm_Kinematics1(2).jnt_limit Arm_Kinematics1(3).jnt_limit Arm_Kinematics1(4).jnt_limit Arm_Kinematics1(5).jnt_limit Arm_Kinematics1(6).jnt_limit];
% num_data = 1;
% sample_step = 20*pi/180;
% for q1 = jnt_limits(1,1):sample_step:jnt_limits(2,1)
%     for q2 = jnt_limits(1,2):sample_step:jnt_limits(2,2)
%         for q3 = jnt_limits(1,3):sample_step:jnt_limits(2,3)
%             for q4 = jnt_limits(1,4):sample_step:jnt_limits(2,4)
%                 for q5 = jnt_limits(1,5):sample_step:jnt_limits(2,5)
%                     q=[q1,q2,q3,q4,q5,0,0,0,0,0,0]';
%                     q_rcm = convert2rcm(q);
%                     Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
%                     rcm_store(:,num_data) = Arm_Kinematics1(18).Tran_matrix(1:3,4);
%                     num_data = num_data + 1;
%                 end
%             end
%         end
%     end
% end
% save('rcm_workspace.mat','rcm_store');

load('rcm_workspace.mat');
num_filtered = 1;
for i = 1 : length(rcm_store)
    if rcm_store(3,i)>0 && rcm_store(2,i)<0
        rcm_filtered(:,num_filtered) = rcm_store(:,i);
        num_filtered = num_filtered + 1;
    end
end
boundary_rcm = boundary(rcm_filtered');
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
trisurf(boundary_rcm,rcm_filtered(1,:)',rcm_filtered(2,:)',rcm_filtered(3,:)','Facecolor','red','FaceAlpha',0.1,'EdgeColor','none');
axis([-1 1 -1 0.5 -0.2 1.2])
grid on

drawnow;