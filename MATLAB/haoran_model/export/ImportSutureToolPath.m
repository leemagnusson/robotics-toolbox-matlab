%% Generate tool path
% Load the log and register the tool path into camera frame.
%%
clc
clear all
close all
% load log
[arm_eef_pose,time_log] = ImportLogfileEEF('C:\Users\haoranyu\Desktop\logs\hernia.log',1);
% init
do_plot = 1;
if do_plot
    figure(1)
    hold on
    view(3)
    axis equal
end
color_tool_path = ['r','g','b','black'];
p_cam = arm_eef_pose(1,1:3,3)';
rotation_cam = QuaternionToRotation(arm_eef_pose(1,4:7,3)');
num_pose = length(arm_eef_pose);
tool_path_left = zeros(4,4,num_pose);
tool_path_right = zeros(4,4,num_pose);
% load and register the left and right arm tool path
for index_sample = 1 : length(arm_eef_pose)
    tool_path_left(:,:,index_sample) = eye(4);
    tool_path_left(1:3,4,index_sample) = rotation_cam' * (arm_eef_pose(index_sample,1:3,1)' - p_cam);
    tool_path_left(1:3,1:3,index_sample) = rotation_cam' * QuaternionToRotation(arm_eef_pose(index_sample,4:7,1)');
    tool_path_right(:,:,index_sample) = eye(4);
    tool_path_right(1:3,4,index_sample) = rotation_cam' * (arm_eef_pose(index_sample,1:3,2)' - p_cam);
    tool_path_right(1:3,1:3,index_sample) = rotation_cam' * QuaternionToRotation(arm_eef_pose(index_sample,4:7,2)');
    
end
% save('export/hernia_tool_path_raw.mat','tool_path_left','tool_path_right','time_log')
if do_plot
    
    scatter3(tool_path_left(1,4,:),tool_path_left(2,4,:),tool_path_left(3,4,:),1,[1 0 0])
    hold on
    scatter3(tool_path_right(1,4,:),tool_path_right(2,4,:),tool_path_right(3,4,:),1,[0 1 0])
    hold on
    
    DrawCoordinateSystem([0.04 0.04 0.04],eye(3),[0;0;0],'rgb','c')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_left(1:3,1:3,1),tool_path_left(1:3,4,1),'rgb','l')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_right(1:3,1:3,1),tool_path_right(1:3,4,1),'rgb','r')
    hold on
    drawnow;
end