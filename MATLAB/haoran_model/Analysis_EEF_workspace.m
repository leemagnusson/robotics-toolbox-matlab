%% EEF workspace analysis
% Explore the robot with fixed RCM and move spherical arm to explore
% workspace
% by Haoran Yu 3/30/2016
%%
clc
clear all
close all
load('/data_store/URDF_info.mat')
load('/data_store/VertexData_origin.mat')
load('/data_store/Arm_version_1.0.mat')
do_plot = 1; % 0 to resweep work space, 1 to just plot
figure(1)
hold on
view(46,23)
axis equal
% setup arm
q=[0,0,0,0,0,0,0,0,0,0,0]';
q_rcm = convert2rcm(q);
base_T=eye(4);
Frames = Arm_class.calc_FK(q_rcm,base_T);
Arm_color = get_arm_color(Arm_class);
Draw_Robot_Arm(Frames,VertexData_origin,Arm_color);
hold on
% input joint limits
jnt_limits = [[-pi;pi] Arm_class(8).jnt_limit Arm_class(11).jnt_limit [-pi;pi] Arm_class(13).jnt_limit Arm_class(14).jnt_limit];
if ~do_plot
    num_data = 1;
    sample_step = 10*pi/180;
    for q6 = jnt_limits(1,1):(sample_step):jnt_limits(2,1)
        for q7 = jnt_limits(1,2):sample_step:jnt_limits(2,2)
            for q8 = jnt_limits(1,3):0.01:jnt_limits(2,3)
                %             for q9 = jnt_limits(1,4):(6*sample_step):jnt_limits(2,4)
                %                 for q10 = jnt_limits(1,5):(6*sample_step):jnt_limits(2,5)
                q=[0,0,0,0,0,q6,q7,q8,0,0,0]';
                q_rcm = convert2rcm(q);
                Frames = Arm_class.calc_FK(q_rcm,base_T);
                eef_store(:,num_data) = Frames(1:3,4,14);
                num_data = num_data + 1;
                %                 end
                %             end
            end
        end
    end
%     save('/data_store/eef_workspace2.mat','eef_store');
else
    load('/data_store/eef_workspace2.mat');
    boundary_eef = boundary(eef_store',0.9);
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    trisurf(boundary_eef,eef_store(1,:)',eef_store(2,:)',eef_store(3,:)','Facecolor','red','FaceAlpha',0.3,'EdgeColor','none');
    axis([-1 1 -1 0.5 -0.2 1.2])
    grid on
    drawnow;
end