%% Startup_haoran
% This code is the startup code to run kinematic analysis in this folder.
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
do_save_file = 1; % 1 to save new mat files; 0 to not save

% First row of parent number is for link in URDF to tell which parent link
% it is attached to. Second row is which joint number defines the
% transformation of this link. For each new URDF with different links and
% joints in order, this array will change.
parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;...
               0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
% Initialize hernia setup
init_Hernia_setup;
% Coupling matrix: The coupling matrix transform the 11 by 1 joint angles
% to 13 by 1 joint angles. The 11 by 1 joint angles are active joint angles
% except for the jaws. The 13 by 1 joint angles add pitch b and pitch c
% joints behind the pitch a joint. A is 13 by 11 with diagnal ones except
% for the coupling components on lines 7 8 9 for pitch a pitch b and pitch
% c joints. q_rcm = A * q;
coupling_matrix = [1 0 0 0 0 0 0 0 0 0 0;...
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

% Load URDF link and joint information
folder_name = 'Arm_version_1.0/';
urdf_file_name = 'V1_Arm_URDF.URDF';
stl_folder_name = strcat(folder_name,'meshes/');
urdf_file_full_name= strcat(folder_name, 'robots/',urdf_file_name);
urdf_input = URDF(urdf_file_full_name);
urdf_link_input = urdf_input.links;
urdf_joint_input = urdf_input.joints;
urdf_joint_sequence = urdf_input.jseq;
vertex_hernia_patient_body = stl2matlab(strcat(stl_folder_name,'Ventral_Hernia_Body.STL'));
scale_mm_to_m = 1/1000;
vertex_hernia_patient_body{1} = vertex_hernia_patient_body{1} * scale_mm_to_m;
vertex_hernia_patient_body{2} = vertex_hernia_patient_body{2} * scale_mm_to_m;
vertex_hernia_patient_body{3} = vertex_hernia_patient_body{3} * scale_mm_to_m;

% Arm kinematics class
arm_kinematics = ArmKinematics(urdf_link_input,urdf_joint_input);

% read stl files
for index_arm=1:length(arm_kinematics)
    if exist(strcat(stl_folder_name,arm_kinematics(index_arm).stl_name_), 'file') == 2
        vertex_arm_origin(:,index_arm)=stl2matlab(strcat(stl_folder_name,arm_kinematics(index_arm).stl_name_));
    else
        vertex_arm_origin(:,index_arm) = {[];[];[]};
    end
    % define index for the coordinates
    if ~isempty(strfind(arm_kinematics(index_arm).name_,'_distal_wrist'))
        index_eef = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_RCM_'))
        index_rcm = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_spherical_roll_'))
        index_car = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_wrist_'))
        index_wrist = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_pitch_a_'))
        index_pitch_a = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_pitch_b_'))
        index_pitch_b = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_pitch_c_'))
        index_pitch_c = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_tool_rotate_'))
        index_tool_rotate = index_arm;
    elseif ~isempty(strfind(arm_kinematics(index_arm).name_,'_tool_translate_'))
        index_tool_translate = index_arm;
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
sample_steps_point_cloud=[100;30;100;30;100;30;30;1;1;1;1;5;100;100;100;100;100;100];
% save point cloud in cell
arm_point_clouds = cell(length(arm_kinematics),1);
arm_point_boundary = cell(length(arm_kinematics),1);
down_sample_threshold = 0.02;
% set base transformation
base_transformation = eye(4);
% initial joint value and convert to rcm joint value
q_init = [0;0;0;0;0;0;0;0;0;0;0];
q_rcm_init = convert2rcm(q_init);
frames_init = arm_kinematics.CalculateFK(q_rcm_init,base_transformation);
%% Generate point clouds
for index_arm = 1:length(arm_kinematics)
    % transformation
    rotation_matrix = frames_init(1:3,1:3,index_arm);
    translation = frames_init(1:3,4,index_arm);
    % check if stl exists and transform
    if isempty(vertex_arm_origin{1,index_arm}) == 0
        % save vertex data
        faces = vertex_arm_origin{1,index_arm};
        vertices = vertex_arm_origin{2,index_arm};
        normals = vertex_arm_origin{3,index_arm};
        % generate point clouds for ith link
        num_point_clouds=0;
        for index = 1 : sample_steps_point_cloud(index_arm) : length(faces)
            % use the center of f v n points
            num_point_clouds = num_point_clouds + 1;
            arm_point_clouds{index_arm,1}(:,num_point_clouds) = 1/3 * ([faces(1,index),vertices(1,index),normals(1,index)] + [faces(2,index),vertices(2,index),normals(2,index)] + [faces(3,index),vertices(3,index),normals(3,index)])';
        end
        % down sample the points on pitch links and translation link by
        % setting the threshold on distance within the points
        if ~isempty(regexp(arm_kinematics(index_arm).name_,'pitch_a||pitch_b||pitch_c||translate','once'))
            for index1 = 1 : 200
                for index2 = 1 : 200
                    if index1 ~= index2 && index1 <= length(arm_point_clouds{index_arm,1}) && index2 <= length(arm_point_clouds{index_arm,1})
                        if norm(arm_point_clouds{index_arm,1}(:,index1)-arm_point_clouds{index_arm,1}(:,index2)) < down_sample_threshold
                            arm_point_clouds{index_arm,1}(:,index2) = [];
                        end
                    end
                end
            end
            if ~isempty(regexp(arm_kinematics(index_arm).name_,'pitch_c','once'))
                length_cur_link = length(arm_point_clouds{index_arm,1});
                arm_point_clouds{index_arm,1}(:,length_cur_link+1) = [0.05689;-0.3214;-0.03313];
                arm_point_clouds{index_arm,1}(:,length_cur_link+2) = [0.05689;-0.3135;-0.07816];
                arm_point_clouds_temp{index_arm,1} = arm_point_clouds{index_arm,1};
                arm_point_clouds{index_arm,1} = [];
                num_point_clouds = 1;
                for index_c = 1 : length(arm_point_clouds_temp{index_arm,1})
                    if arm_point_clouds_temp{index_arm,1}(2,index_c)<=0.03
                        arm_point_clouds{index_arm,1}(:,num_point_clouds) = arm_point_clouds_temp{index_arm,1}(:,index_c);
                        num_point_clouds = num_point_clouds + 1;
                    end
                end
            end
        end
        if ~isempty(arm_point_clouds{index_arm,1})
            [rotmat,arm_point_boundary{index_arm,1},volume,surface,edgelength] = minboundbox(arm_point_clouds{index_arm,1}(1,:),arm_point_clouds{index_arm,1}(2,:),arm_point_clouds{index_arm,1}(3,:),'e');
        end
    end
end

%% plot point clouds and arm
for index_arm = 1:length(arm_kinematics)
    rotation_matrix = frames_init(1:3,1:3,index_arm);
    translation = frames_init(1:3,4,index_arm);
    if isempty(vertex_arm_origin{1,index_arm}) == 0
        % transform vertex data
        vertex_arm_transformed(:,index_arm) = transformSTL(vertex_arm_origin(:,index_arm),rotation_matrix,translation);
        arm_color = arm_kinematics(index_arm).color_;
        for index = 1 : length(arm_point_clouds{index_arm,1})
            % transform point clouds
            arm_point_clouds_transformed = rotation_matrix * arm_point_clouds{index_arm}(:,index) + translation;
            plot3(arm_point_clouds_transformed(1),arm_point_clouds_transformed(2),arm_point_clouds_transformed(3),'Marker','o')
            hold on
        end
        for index = 1 : length(arm_point_boundary{index_arm,1})
            % transform point boundary corners
            arm_point_boundary{index_arm,1}(index,:) = (rotation_matrix * arm_point_boundary{index_arm}(index,:)' + translation)';
        end
        plotminbox(arm_point_boundary{index_arm,1});
        hold on
        plotSTL(vertex_arm_transformed(:,index_arm),arm_color)
        hold on
    end
end
axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
drawnow;

% Change do_save to 1 if user want to regenerate the data files. This
% process is usually for new URDFs and new Mesh files.
if do_save_file
    save('data_store/parent_number.mat','parent_number'); % save parent and joint number for current URDF
    save('data_store/coupling_matrix.mat','coupling_matrix'); % save coupling matrix
    save('data_store/urdf_info.mat','urdf_link_input','urdf_joint_input','urdf_joint_sequence'); % save URDF for the arm
    save('data_store/arm_version_1.0.mat','arm_kinematics') % save the arm kinematics class that contains the arm kinematic information
    save('data_store/index_joints.mat','index_eef','index_rcm','index_car','index_wrist','index_pitch_a','index_pitch_b','index_pitch_c','index_tool_rotate','index_tool_translate'); % save the index of joints
    save('data_store/vertex_arm_origin.mat','vertex_arm_origin'); % save the vertex data of the arm for plot
    save('data_store/vertex_hernia_patient_body.mat','vertex_hernia_patient_body'); % save the vertex data of the hernia body for plot
    save ('data_store/arm_point_clouds.mat','arm_point_clouds'); % save the point clouds of the arm
    save ('data_store/arm_point_boundary.mat','arm_point_boundary'); % save the bounding box of the arm
end