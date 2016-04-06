%% Arm kinematics class definition
% Copyright Verb Surgical Inc.
% Arm Kinematics is a class we generate to take the input from:
% link_input and joint_input from URDF file: '****.URDF'
% name: name of the link and joint pair
% stl_name: name of the stl
% mass: mass of the link
% center_of_mass: center of mass of the link
% inertia_matrix: inertia matrix of the link
% color: color of the link
% parent: parent link/joint pair
% joint_limit: joint limit
% rpy: roll pitch yaw angles
% xyz: translation from parent
% joint_axis: active rotation joint axis
% type: link type
% ArmKinematics.calc_FK: calculate forward kinematics and return Frames
% with 4 by 4 by n format.

%%

classdef ArmKinematics
    properties
        name_ % name of the link and joint pair
        stl_name_ % name of the stl
        mass_ % mass of the link
        center_of_mass_ % center of mass of the link
        inertia_matrix_ % inertia matrix of the link
        color_ % color of the link
        parent_ % parent link/joint pair
        joint_limit_ % joint limit
        rpy_ % roll pitch yaw angles
        xyz_ % translation from parent
        joint_axis_ % active rotation joint axis
        joint_type_ % link type
    end
    methods
        function ArmKinematics = ArmKinematics(urdf_link_input,urdf_joint_input)
            % load the parent number and joint number.
            load('/../data_store/parent_number.mat');
            if nargin ~=0
                ArmKinematics(length(urdf_link_input)) = ArmKinematics;
                for index_link = 1:length(urdf_link_input)
                    % set parent and joint number from the mat file
                    joint_number_index = parent_number(2,index_link);
                    ArmKinematics(index_link).parent_ = parent_number(1,index_link);
                    % set the color if link input has 'visual' field
                    if isfield(urdf_link_input{index_link},'visual')
                        ArmKinematics(index_link).color_ = urdf_link_input{index_link}.visual.material.color.rgba;
                    else
                        ArmKinematics(index_link).color_ = [1 1 1 1];
                    end
                    % set the name following the link name
                    ArmKinematics(index_link).name_ = urdf_link_input{index_link}.name;
                    % Define joint limits
                    if index_link==1
                        % base link is initialized differently
                        ArmKinematics(index_link).joint_limit_ = [[];[]];
                        ArmKinematics(index_link).xyz_ = [0;0;0];
                        ArmKinematics(index_link).rpy_ = [0;0;0];
                        ArmKinematics(index_link).joint_axis_ = [0;0;1];
                        ArmKinematics(index_link).joint_type_ = 'fixed';
                    else
                        % detect fixed links and set link type andd active
                        % joint axis
                        if strcmp(urdf_joint_input{joint_number_index}.type, 'fixed') || ~isempty(strfind(urdf_joint_input{joint_number_index}.name,'jaw'))
                            ArmKinematics(index_link).joint_type_ = 'fixed';
                            ArmKinematics(index_link).joint_axis_ = [0;0;1];
                        else
                            ArmKinematics(index_link).joint_type_ = urdf_joint_input{joint_number_index}.type;
                            ArmKinematics(index_link).joint_axis_ = urdf_joint_input{joint_number_index}.axis.xyz';
                        end
                        
                        % set the xyz translation and rpy euler angles
                        ArmKinematics(index_link).xyz_ = urdf_joint_input{joint_number_index}.origin.xyz';
                        ArmKinematics(index_link).rpy_ = urdf_joint_input{joint_number_index}.origin.rpy';
                        % get joint limits
                        if isfield(urdf_joint_input{joint_number_index},'limit')
                            ArmKinematics(index_link).joint_limit_ = [urdf_joint_input{joint_number_index}.limit.lower; urdf_joint_input{joint_number_index}.limit.upper];
                        else
                            ArmKinematics(index_link).joint_limit_ = [[];[]];
                        end
                    end
                    % set stl file name
                    ArmKinematics(index_link).stl_name_ = strcat(urdf_link_input{index_link}.name,'.stl');
                    % check and set inertial terms for dynamics
                    if isfield(urdf_link_input{index_link},'inertial')
                        ArmKinematics(index_link).mass_ = urdf_link_input{index_link}.inertial.mass.value;
                        ArmKinematics(index_link).center_of_mass_ = urdf_link_input{index_link}.inertial.origin.xyz;
                        ArmKinematics(index_link).inertia_matrix_ = [urdf_link_input{index_link}.inertial.inertia.ixx urdf_link_input{index_link}.inertial.inertia.ixy urdf_link_input{index_link}.inertial.inertia.ixz;...
                            urdf_link_input{index_link}.inertial.inertia.ixy urdf_link_input{index_link}.inertial.inertia.iyy urdf_link_input{index_link}.inertial.inertia.iyz;...
                            urdf_link_input{index_link}.inertial.inertia.ixz urdf_link_input{index_link}.inertial.inertia.iyz urdf_link_input{index_link}.inertial.inertia.izz];
                    else
                        ArmKinematics(index_link).mass_ = 0;
                        ArmKinematics(index_link).center_of_mass_ = 0;
                        ArmKinematics(index_link).inertia_matrix_ = zeros(3,3);
                    end
                end
            end
        end
        
        function robot_frames = CalculateFK(arm_kinematics,q_rcm,robot_base_transform)
            load('/../data_store/parent_number.mat');
            if nargin ~=0
                for index_arm = 1:length(arm_kinematics)
                    % set parent and joint number from the mat file
                    joint_index = parent_number(2,index_arm);
                    % Define homogeneous transformation
                    if index_arm==1
                        % base link
                        robot_frames(:,:,index_arm) = robot_base_transform;
                    else
                        % joint translation and fixed frame roll pitch yaw
                        frame_translation = arm_kinematics(index_arm).xyz_;
                        frame_rotation_euler = arm_kinematics(index_arm).rpy_;
                        joint_axis = arm_kinematics(index_arm).joint_axis_;
                        % active joint value
                        if strcmp(arm_kinematics(index_arm).joint_type_, 'fixed')
                            joint_value = 0;
                        else
                            joint_value = q_rcm(joint_index);
                        end
                        % homogeneous transformation for revolute and
                        % prismatic joints
                        if ~strcmp(arm_kinematics(index_arm).joint_type_, 'prismatic')
                            frame_rotation_matrix = rotr([0;0;1],frame_rotation_euler(3))*...
                                rotr([0;1;0],frame_rotation_euler(2))*...
                                rotr([1;0;0],frame_rotation_euler(1))*...
                                rotr(joint_axis,joint_value);
                            robot_frames(:,:,index_arm) = [frame_rotation_matrix frame_translation; 0 0 0 1];
                        else
                            frame_rotation_matrix = rotr([0;0;1],frame_rotation_euler(3))*...
                                rotr([0;1;0],frame_rotation_euler(2))*...
                                rotr([1;0;0],frame_rotation_euler(1));
                            frame_displacement = joint_axis * joint_value;
                            robot_frames(:,:,index_arm) = [frame_rotation_matrix frame_translation + frame_rotation_matrix * frame_displacement; 0 0 0 1];
                        end
                    end
                end
                % Transform the homonegeous matrices to world frame
                for index_arm = 2:length(arm_kinematics)
                    robot_frames(:,:,index_arm) = robot_frames(:,:,arm_kinematics(index_arm).parent_) * robot_frames(:,:,index_arm);
                end
            end
        end
    end
end
