%% Generate tool path for stylus sponge
% Run 'ImportLogSponge.m' first to generate the mat file for this script.
% The script taks the raw quaternion for arm 1-4 poses and register them
% into the camera frame (third arm (arm2)). The fixed camera frame is the
% first frame in the camera frame sequence.
%%
% clc
% clear all
close all
% load log
ImportLogSponge;
time_log = time_stamp;
% init
do_plot = 1;
if do_plot
    figure(1)
    hold on
    view(3)
    axis equal
end
color_tool_path = ['r','g','b','black'];
p_camera_init = arm2(1,1:3)';
rotation_camera_init = QuaternionToRotation(arm2(1,4:7)');
% rotation_camera_init = eye(3);
% p_camera_init = [0;0;0];
num_pose = length(time_stamp);
tool_path_left = zeros(4,4,num_pose);
tool_path_right = zeros(4,4,num_pose);

gripper_left = gripper0';
gripper_right = gripper1';
gripper_camera = gripper2';

% load and register the left and right arm tool path
for index_sample = 1 : num_pose
    
    tool_path_camera(:,:,index_sample) = eye(4);
    p_camera_cur = arm2(index_sample,1:3)';
    rotation_camera_cur = QuaternionToRotation(arm2(index_sample,4:7)');
    tool_path_camera(1:3,4,index_sample) = rotation_camera_init'*(p_camera_cur - p_camera_init);
    tool_path_camera(1:3,1:3,index_sample) = rotation_camera_init' * rotation_camera_cur;
    
    p_left_cur = arm0(index_sample,1:3)';
    rotation_left_cur = QuaternionToRotation(arm0(index_sample,4:7)');
    tool_path_left(:,:,index_sample) = eye(4);
    tool_path_left(1:3,4,index_sample) = rotation_camera_init' * (p_left_cur - p_camera_init);
    tool_path_left(1:3,1:3,index_sample) = rotation_camera_init' * rotation_left_cur;

    p_right_cur = arm1(index_sample,1:3)';
    rotation_right_cur = QuaternionToRotation(arm1(index_sample,4:7)');
    tool_path_right(:,:,index_sample) = eye(4);
    tool_path_right(1:3,4,index_sample) = rotation_camera_init' * (p_right_cur - p_camera_init);
    tool_path_right(1:3,1:3,index_sample) = rotation_camera_init' * rotation_right_cur;

end
% save('export/egg_sponge_user04_tool_path_raw_with_angle.mat','tool_path_left','tool_path_right','tool_path_camera','time_log','gripper_left','gripper_right','gripper_camera')
save(data_name1,'tool_path_left','tool_path_right','tool_path_camera','time_log','gripper_left','gripper_right','gripper_camera')
if do_plot
    
    scatter3(tool_path_left(1,4,:),tool_path_left(2,4,:),tool_path_left(3,4,:),1,[1 0 0])
    hold on
    scatter3(tool_path_right(1,4,:),tool_path_right(2,4,:),tool_path_right(3,4,:),1,[0 1 0])
    hold on
    
    scatter3(tool_path_camera(1,4,:),tool_path_camera(2,4,:),tool_path_camera(3,4,:),1,[0 0 1])
    hold on
    
    DrawCoordinateSystem([0.04 0.04 0.04],eye(3),[0;0;0],'rgb','c')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_left(1:3,1:3,1),tool_path_left(1:3,4,1),'rgb','l')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_right(1:3,1:3,1),tool_path_right(1:3,4,1),'rgb','r')
    hold on
    DrawCoordinateSystem([0.02 0.02 0.02],tool_path_camera(1:3,1:3,1),tool_path_camera(1:3,4,1),'rgb','c')
    hold on
    drawnow;
end