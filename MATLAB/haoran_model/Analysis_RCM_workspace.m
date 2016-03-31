%% RCM workspace analysis
% Explore the RCM workspace of the arm by controlling joint 1 to 5
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
jnt_limits = [Arm_class(2).jnt_limit Arm_class(3).jnt_limit Arm_class(4).jnt_limit Arm_class(5).jnt_limit Arm_class(6).jnt_limit];
if ~do_plot
    num_data = 1;
    sample_step = 20*pi/180;
    for q1 = jnt_limits(1,1):sample_step:jnt_limits(2,1)
        for q2 = jnt_limits(1,2):sample_step:jnt_limits(2,2)
            for q3 = jnt_limits(1,3):sample_step:jnt_limits(2,3)
                for q4 = jnt_limits(1,4):sample_step:jnt_limits(2,4)
                    for q5 = jnt_limits(1,5):sample_step:jnt_limits(2,5)
                        q=[q1,q2,q3,q4,q5,0,0,0,0,0,0]';
                        q_rcm = convert2rcm(q);
                        Frames = Arm_class.calc_FK(q_rcm,base_T);
                        rcm_store(:,num_data) = Frames(1:3,4,18);
                        num_data = num_data + 1;
                    end
                end
            end
        end
    end
%     save('/data_store/rcm_workspace.mat','rcm_store');
else
    load('/data_store/rcm_workspace.mat');
    % filter work space
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
end