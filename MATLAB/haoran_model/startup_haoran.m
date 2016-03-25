%% Startup_haoran
% This code is the startup code to run kinematic analysis in this folder.
% % by Haoran Yu 3/16/2016
%% initialization
clc
clear all
close all
ARM_setup_hernia
% Add path for useful libraries
addpath([pwd, '/../3rdparty/robotics-toolbox-matlab']);
% First row of parent number is for link in URDF to tell which parent link
% it is attached to. Second row is which joint number defines the
% transformation of this link.
parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;...
    0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
% save('parent_number.mat','parent_number'); % uncomment for a new URDF

A = [1 0 0 0 0 0 0 0 0 0 0;...
    0 1 0 0 0 0 0 0 0 0 0;...
    0 0 1 0 0 0 0 0 0 0 0;...
    0 0 0 1 0 0 0 0 0 0 0;...
    0 0 0 0 1 0 0 0 0 0 0;...
    0 0 0 0 0 1 0 0 0 0 0;...
    0 0 0 0 0 0 1 0 0 0 0;...
    0 0 0 0 0 0 -1 0 0 0 0;...
    0 0 0 0 0 0 1 0 0 0 0;...
    0 0 0 0 0 0 0 1 0 0 0;...
    0 0 0 0 0 0 0 0 1 0 0;...
    0 0 0 0 0 0 0 0 0 1 0;...
    0 0 0 0 0 0 0 0 0 0 1];
% save('coupling_matrix.mat','A');

% Load URDF link and joint information
folder_name = 'Arm_version_1.0/';
URDF_name = 'V1_Arm_URDF.URDF';
stl_folder_name = strcat(folder_name,'meshes/');
URDF_file= strcat(folder_name, 'robots/',URDF_name);
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;
joint_sequence = urdf_input.jseq;
save('URDF_info.mat','link_input','joint_input','joint_sequence'); % uncomment for a new URDF
VertexData_Hernia_Body = stl2matlab(strcat(stl_folder_name,'Ventral_Hernia_Body.STL'));
VertexData_Hernia_Body{1} = VertexData_Hernia_Body{1}/1000;
VertexData_Hernia_Body{2} = VertexData_Hernia_Body{2}/1000;
VertexData_Hernia_Body{3} = VertexData_Hernia_Body{3}/1000;
% set base transformation
base_T = eye(4);
% initial joint value and convert to rcm joint value
q_init = [0;0;0;0;0;0;0;0;0;0;0];
q_rcm_init = convert2rcm(q_init);
% initial arm kinematics
Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm_init,base_T);
% read stl files
for i=1:length(Arm_Kinematics_init)
    if exist(strcat(stl_folder_name,Arm_Kinematics_init(i).stl_name), 'file') == 2
        VertexData_origin(:,i)=stl2matlab(strcat(stl_folder_name,Arm_Kinematics_init(i).stl_name));
    else
        VertexData_origin(:,i) = {[];[];[]};
    end
    % define index for the coordinates
    if ~isempty(strfind(Arm_Kinematics_init(i).name,'_distal_wrist'))
        index_eef = i;
    elseif ~isempty(strfind(Arm_Kinematics_init(i).name,'_RCM_'))
        index_rcm = i;
    elseif ~isempty(strfind(Arm_Kinematics_init(i).name,'_spherical_roll_'))
        index_car = i;
    elseif ~isempty(strfind(Arm_Kinematics_init(i).name,'_wrist_'))
        index_wrist = i;
    elseif ~isempty(strfind(Arm_Kinematics_init(i).name,'_pitch_c_'))
        index_pitch_c = i;
    elseif ~isempty(strfind(Arm_Kinematics_init(i).name,'_tool_rotate_'))
        index_tool_rotate = i;
    elseif ~isempty(strfind(Arm_Kinematics_init(i).name,'_tool_translate_'))
        index_tool_translate = i;
    end
end
% save('index_joints.mat','index_eef','index_rcm','index_car','index_wrist','index_pitch_c','index_tool_rotate','index_tool_translate');
% save('VertexData_origin.mat','VertexData_origin'); % uncomment for a new urdf
% save('VertexData_Hernia_Body.mat','VertexData_Hernia_Body'); % 
fclose('all');

%% generate the point cloud for collision detection
%
figure(1)
hold on
view(62,28)
axis equal
% set the sampling step for reading the point cloud on each link.
% 18 link in total.
steps=[100;30;100;30;100;30;30;1;1;1;1;5;100;100;100;100;100;100];
% save point cloud in cell
point_clouds = cell(length(Arm_Kinematics_init),1);
down_sample_thres = 0.02;
%% Generate point clouds
for i = 1:length(Arm_Kinematics_init)
    % transformation
    R = Arm_Kinematics_init(i).Tran_matrix(1:3,1:3);
    d = Arm_Kinematics_init(i).Tran_matrix(1:3,4);
    % check if stl exists and transform
    if isempty(VertexData_origin{1,i}) == 0
        % save vertex data
        f = VertexData_origin{1,i};
        v = VertexData_origin{2,i};
        n = VertexData_origin{3,i};
        % generate point clouds for ith link
        number=1;
        for index = 1 : steps(i) : length(f)
            % use the center of f v n points
            point_clouds{i,1}(:,number) = 1/3 * ([f(1,index),v(1,index),n(1,index)] + [f(2,index),v(2,index),n(2,index)] + [f(3,index),v(3,index),n(3,index)])';
            number = number + 1;
        end
        % down sample the points on pitch links and translation link by
        % setting the threshold on distance within the points
        if ~isempty(regexp(Arm_Kinematics_init(i).name,'pitch_a||pitch_b||pitch_c||translate','once'))
            for index1 = 1 : 200
                for index2 = 1 : 200
                    if index1 ~= index2 && index1 <= length(point_clouds{i,1}) && index2 <= length(point_clouds{i,1})
                        if norm(point_clouds{i,1}(:,index1)-point_clouds{i,1}(:,index2)) < down_sample_thres
                            point_clouds{i,1}(:,index2) = [];
                            number =number +1;
                        end
                    end
                end
            end
        end
    end
end
% save ('point_clouds_all.mat','point_clouds'); % uncomment for a new urdf

%% plot point clouds and arm
for i = 1:length(Arm_Kinematics_init)
    R = Arm_Kinematics_init(i).Tran_matrix(1:3,1:3);
    d = Arm_Kinematics_init(i).Tran_matrix(1:3,4);
    if isempty(VertexData_origin{1,i}) == 0
        % transform vertex data
        VertexData_tran(:,i) = transformSTL(VertexData_origin(:,i),R,d);
        rgba = Arm_Kinematics_init(i).color;
        for index = 1 : length(point_clouds{i,1})
            % transform point clouds
            center = R * point_clouds{i}(:,index) + d;
            plot3(center(1),center(2),center(3),'Marker','o')
            hold on
        end
        plotSTL(VertexData_tran(:,i),rgba)
        hold on
    end
end
axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
drawnow;
