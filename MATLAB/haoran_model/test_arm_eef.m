clc
clear all
close all
[eef_arm,time] = import_logfile_eef('C:\Users\haoranyu\Desktop\logs\hernia.log',10);
figure(1)
hold on
view(3)
axis equal
color = ['r','g','b','black'];
p_cam = eef_arm(1,1:3,3)';
R_cam = quat2rot_ros(eef_arm(1,4:7,3)');
% p_cam = [0;0;0];
% R_cam = eye(3);
num = length(eef_arm);
Tool_path_left = zeros(4,4,num);
Tool_path_right = zeros(4,4,num);
for j = 1 : length(eef_arm)
    Tool_path_left(:,:,j) = eye(4);
    Tool_path_left(1:3,4,j) = R_cam' * (eef_arm(j,1:3,1)' - p_cam);
    Tool_path_left(1:3,1:3,j) = R_cam' * quat2rot_ros(eef_arm(j,4:7,1)');
    Tool_path_right(:,:,j) = eye(4);
    Tool_path_right(1:3,4,j) = R_cam' * (eef_arm(j,1:3,2)' - p_cam);
    Tool_path_right(1:3,1:3,j) = R_cam' * quat2rot_ros(eef_arm(j,4:7,2)');
    plot3(Tool_path_left(1,4,j),Tool_path_left(2,4,j),Tool_path_left(3,4,j),'Color',color(1),'Marker','o')
    hold on
    plot3(Tool_path_right(1,4,j),Tool_path_right(2,4,j),Tool_path_right(3,4,j),'Color',color(2),'Marker','o')
    hold on
end
% save('Tool_path.mat','Tool_path_left','Tool_path_right','time')
draw_coordinate_system([0.04 0.04 0.04],eye(3),[0;0;0],'rgb','c')
hold on

draw_coordinate_system([0.02 0.02 0.02],Tool_path_left(1:3,1:3,1),Tool_path_left(1:3,4,1),'rgb','l')
hold on

draw_coordinate_system([0.02 0.02 0.02],Tool_path_right(1:3,1:3,1),Tool_path_right(1:3,4,1),'rgb','r')
hold on
drawnow;