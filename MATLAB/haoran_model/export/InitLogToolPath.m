%% Generate tool path
% Load the log "hernia.log" and register the tool path into camera frame.
% the log will be loaded to generate quaternions for poses in all four
% arms. the first and third arms are left and right arms. The poses are
% registered into the camera frame provided by the second arm. The tool
% path with the time log is saved into "hernia_tool_path.mat". The log is
% available on google drive at
% https://drive.google.com/drive/u/0/folders/0B3-rJB5tIf07RTFFTU1UcFE5LWs
%%
clc
clear all
close all
% load log
[arm_eef_pose,time_log] = ImportLogfileEEF('C:\Users\haoranyu\Desktop\logs\hernia.log',10);
% init
do_plot = 0;
if do_plot
    figure(1)
    hold on
    view(3)
    axis equal
end
color = ['r','g','b','black'];
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
    if do_plot
        plot3(tool_path_left(1,4,index_sample),tool_path_left(2,4,index_sample),tool_path_left(3,4,index_sample),'Color',color(1),'Marker','o')
        hold on
        plot3(tool_path_right(1,4,index_sample),tool_path_right(2,4,index_sample),tool_path_right(3,4,index_sample),'Color',color(2),'Marker','o')
        hold on
    end
end
save('data/hernia_tool_path.mat','tool_path_left','tool_path_right','time_log')
if do_plot
    DrawCoordinateSystem([0.04 0.04 0.04],eye(3),[0;0;0],'rgb','c')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_left(1:3,1:3,1),tool_path_left(1:3,4,1),'rgb','l')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_right(1:3,1:3,1),tool_path_right(1:3,4,1),'rgb','r')
    hold on
    drawnow;
end