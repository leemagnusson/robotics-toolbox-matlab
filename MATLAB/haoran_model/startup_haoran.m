%% Startup_haoran
% This code is the startup code to run kinematic analysis in this folder.
% % by Haoran Yu 3/16/2016
%% initialization
clc
clear all
close all
% Add path for useful libraries
addpath([pwd, '/../3rdparty/robotics-toolbox-matlab']);
addpath([pwd, '/Kinematics_library']);
addpath([pwd, '/Math_library']);
addpath([pwd, '/Plot_library']);
addpath([pwd, '/Collision_detection_library']);
addpath([pwd, '/Common_library']);
addpath([pwd, '/data_store']);
do_save = 1; % 1 to save new mat files; 0 to not save

% First row of parent number is for link in URDF to tell which parent link
% it is attached to. Second row is which joint number defines the
% transformation of this link.
parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;...
    0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
% save('/data_store/parent_number.mat','parent_number'); % uncomment for a new URDF
init_Hernia_setup;
% coupling matrix
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
% save('/data_store/coupling_matrix.mat','A');

% Load URDF link and joint information
folder_name = 'Arm_version_1.0_modified/';
URDF_name = 'V1_Arm_URDF.URDF';
stl_folder_name = strcat(folder_name,'meshes/');
URDF_file= strcat(folder_name, 'robots/',URDF_name);
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;
joint_sequence = urdf_input.jseq;
VertexData_Hernia_Body = stl2matlab(strcat(stl_folder_name,'Ventral_Hernia_Body.STL'));
VertexData_Hernia_Body{1} = VertexData_Hernia_Body{1}/1000;
VertexData_Hernia_Body{2} = VertexData_Hernia_Body{2}/1000;
VertexData_Hernia_Body{3} = VertexData_Hernia_Body{3}/1000;

% Arm kinematics class
Arm_class = Arm_Kinematics(link_input,joint_input);

% read stl files
for i=1:length(Arm_class)
    if exist(strcat(stl_folder_name,Arm_class(i).stl_name), 'file') == 2
        VertexData_origin(:,i)=stl2matlab(strcat(stl_folder_name,Arm_class(i).stl_name));
    else
        VertexData_origin(:,i) = {[];[];[]};
    end
    % define index for the coordinates
    if ~isempty(strfind(Arm_class(i).name,'_distal_wrist'))
        index_eef = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_RCM_'))
        index_rcm = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_spherical_roll_'))
        index_car = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_wrist_'))
        index_wrist = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_pitch_a_'))
        index_pitch_a = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_pitch_b_'))
        index_pitch_b = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_pitch_c_'))
        index_pitch_c = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_tool_rotate_'))
        index_tool_rotate = i;
    elseif ~isempty(strfind(Arm_class(i).name,'_tool_translate_'))
        index_tool_translate = i;
    end
end
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
point_clouds = cell(length(Arm_class),1);
point_boundary = cell(length(Arm_class),1);
down_sample_thres = 0.02;
% set base transformation
base_T = eye(4);
% initial joint value and convert to rcm joint value
q_init = [0;0;0;0;0;0;0;0;0;0;0];
q_rcm_init = convert2rcm(q_init);
Frames_init = Arm_class.calc_FK(q_rcm_init,base_T);
%% Generate point clouds
for i = 1:length(Arm_class)
    % transformation
    R = Frames_init(1:3,1:3,i);
    d = Frames_init(1:3,4,i);
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
        if ~isempty(regexp(Arm_class(i).name,'pitch_a||pitch_b||pitch_c||translate','once'))
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
            if ~isempty(regexp(Arm_class(i).name,'pitch_c','once'))
                length_cur_link = length(point_clouds{i,1});
                point_clouds{i,1}(:,length_cur_link+1) = [0.05689;-0.3214;-0.03313];
                point_clouds{i,1}(:,length_cur_link+2) = [0.05689;-0.3135;-0.07816];
                point_clouds_temp{i,1} = point_clouds{i,1};
                point_clouds{i,1} = [];
                number = 1;
                for index_c = 1 : length(point_clouds_temp{i,1})
                    if point_clouds_temp{i,1}(2,index_c)<=0.03
                        point_clouds{i,1}(:,number) = point_clouds_temp{i,1}(:,index_c);
                        number = number + 1;
                    end
                end
            end
        end
        if ~isempty(point_clouds{i,1})
            
            [rotmat,point_boundary{i,1},volume,surface,edgelength] = minboundbox(point_clouds{i,1}(1,:),point_clouds{i,1}(2,:),point_clouds{i,1}(3,:),'e');
        end
    end
end

%% plot point clouds and arm
for i = 1:length(Arm_class)
    R = Frames_init(1:3,1:3,i);
    d = Frames_init(1:3,4,i);
    if isempty(VertexData_origin{1,i}) == 0
        % transform vertex data
        VertexData_tran(:,i) = transformSTL(VertexData_origin(:,i),R,d);
        rgba = Arm_class(i).color;
        for index = 1 : length(point_clouds{i,1})
            % transform point clouds
            center = R * point_clouds{i}(:,index) + d;
            plot3(center(1),center(2),center(3),'Marker','o')
            hold on
            
        end
        for index = 1 : length(point_boundary{i,1})
            % transform point boundary corners
            point_boundary{i,1}(index,:) = (R * point_boundary{i}(index,:)' + d)';
        end
        plotminbox(point_boundary{i,1});
        hold on
        plotSTL(VertexData_tran(:,i),rgba)
        hold on
    end
end
axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
drawnow;
if do_save
    save('data_store/URDF_info.mat','link_input','joint_input','joint_sequence'); % uncomment for a new URDF
    save('data_store/Arm_version_1.0.mat','Arm_class') % uncomment for a new URDF
    save('data_store/index_joints.mat','index_eef','index_rcm','index_car','index_wrist','index_pitch_a','index_pitch_b','index_pitch_c','index_tool_rotate','index_tool_translate');
    save('data_store/VertexData_origin.mat','VertexData_origin'); % uncomment for a new urdf
    save('data_store/VertexData_Hernia_Body.mat','VertexData_Hernia_Body'); %
    save ('data_store/point_clouds_all.mat','point_clouds'); % uncomment for a new urdf
    save ('data_store/point_boundary_all.mat','point_boundary');
end