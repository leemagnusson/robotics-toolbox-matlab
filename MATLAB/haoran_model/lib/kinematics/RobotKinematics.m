%% Robot kinematics class definition
% Copyright Verb Surgical Inc.
% Robot Kinematics is a class we generate to take the input from:
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
% RobotKinematics.CalculateFK: calculate forward kinematics and return Frames
% with 4 by 4 by n format.

%%

classdef RobotKinematics
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
        function robot_kinematics = RobotKinematics(urdf_link_input,urdf_joint_input)
            % load the parent number and joint number.
            load('parent_number.mat');
            if nargin ~=0
                robot_kinematics(length(urdf_link_input)) = robot_kinematics;
                for index_link = 1:length(urdf_link_input)
                    % set parent and joint number from the mat file
                    joint_number_index = parent_number(2,index_link);
                    robot_kinematics(index_link).parent_ = parent_number(1,index_link);
                    % set the color if link input has 'visual' field
                    if isfield(urdf_link_input{index_link},'visual')
                        robot_kinematics(index_link).color_ = urdf_link_input{index_link}.visual.material.color.rgba;
                    else
                        robot_kinematics(index_link).color_ = [1 1 1 1];
                    end
                    % set the name following the link name
                    robot_kinematics(index_link).name_ = urdf_link_input{index_link}.name;
                    % Define joint limits
                    if index_link==1
                        % base link is initialized differently
                        robot_kinematics(index_link).joint_limit_ = [[];[]];
                        robot_kinematics(index_link).xyz_ = [0;0;0];
                        robot_kinematics(index_link).rpy_ = [0;0;0];
                        robot_kinematics(index_link).joint_axis_ = [0;0;1];
                        robot_kinematics(index_link).joint_type_ = 'fixed';
                    else
                        % detect fixed links and set link type andd active
                        % joint axis
                        if strcmp(urdf_joint_input{joint_number_index}.type, 'fixed') || ~isempty(strfind(urdf_joint_input{joint_number_index}.name,'jaw'))
                            robot_kinematics(index_link).joint_type_ = 'fixed';
                            robot_kinematics(index_link).joint_axis_ = [0;0;1];
                        else
                            robot_kinematics(index_link).joint_type_ = urdf_joint_input{joint_number_index}.type;
                            robot_kinematics(index_link).joint_axis_ = urdf_joint_input{joint_number_index}.axis.xyz';
                        end
                        
                        % set the xyz translation and rpy euler angles
                        robot_kinematics(index_link).xyz_ = urdf_joint_input{joint_number_index}.origin.xyz';
                        robot_kinematics(index_link).rpy_ = urdf_joint_input{joint_number_index}.origin.rpy';
                        % get joint limits
                        if isfield(urdf_joint_input{joint_number_index},'limit')
                            robot_kinematics(index_link).joint_limit_ = [urdf_joint_input{joint_number_index}.limit.lower; urdf_joint_input{joint_number_index}.limit.upper];
                        else
                            robot_kinematics(index_link).joint_limit_ = [[];[]];
                        end
                    end
                    % set stl file name
                    robot_kinematics(index_link).stl_name_ = strcat(urdf_link_input{index_link}.name,'.stl');
                    % check and set inertial terms for dynamics
                    if isfield(urdf_link_input{index_link},'inertial')
                        robot_kinematics(index_link).mass_ = urdf_link_input{index_link}.inertial.mass.value;
                        robot_kinematics(index_link).center_of_mass_ = urdf_link_input{index_link}.inertial.origin.xyz;
                        robot_kinematics(index_link).inertia_matrix_ = [urdf_link_input{index_link}.inertial.inertia.ixx urdf_link_input{index_link}.inertial.inertia.ixy urdf_link_input{index_link}.inertial.inertia.ixz;...
                            urdf_link_input{index_link}.inertial.inertia.ixy urdf_link_input{index_link}.inertial.inertia.iyy urdf_link_input{index_link}.inertial.inertia.iyz;...
                            urdf_link_input{index_link}.inertial.inertia.ixz urdf_link_input{index_link}.inertial.inertia.iyz urdf_link_input{index_link}.inertial.inertia.izz];
                    else
                        robot_kinematics(index_link).mass_ = 0;
                        robot_kinematics(index_link).center_of_mass_ = 0;
                        robot_kinematics(index_link).inertia_matrix_ = zeros(3,3);
                    end
                end
            end
        end
        
        function frames_robot = CalculateFK(robot_kinematics,q_rcm,robot_base_transform)
            load('parent_number.mat');
            if nargin ~=0
                for index_arm = 1:length(robot_kinematics)
                    % set parent and joint number from the mat file
                    joint_index = parent_number(2,index_arm);
                    % Define homogeneous transformation
                    if index_arm==1
                        % base link
                        frames_robot(:,:,index_arm) = robot_base_transform;
                    else
                        % joint translation and fixed frame roll pitch yaw
                        frame_translation = robot_kinematics(index_arm).xyz_;
                        frame_rotation_euler = robot_kinematics(index_arm).rpy_;
                        joint_axis = robot_kinematics(index_arm).joint_axis_;
                        % active joint value
                        if strcmp(robot_kinematics(index_arm).joint_type_, 'fixed')
                            joint_value = 0;
                        else
                            joint_value = q_rcm(joint_index);
                        end
                        % homogeneous transformation for revolute and
                        % prismatic joints
                        if ~strcmp(robot_kinematics(index_arm).joint_type_, 'prismatic')
                            frame_rotation_matrix = RotationAxisAngle([0;0;1],frame_rotation_euler(3))*...
                                RotationAxisAngle([0;1;0],frame_rotation_euler(2))*...
                                RotationAxisAngle([1;0;0],frame_rotation_euler(1))*...
                                RotationAxisAngle(joint_axis,joint_value);
                            frames_robot(:,:,index_arm) = [frame_rotation_matrix frame_translation; 0 0 0 1];
                        else
                            frame_rotation_matrix = RotationAxisAngle([0;0;1],frame_rotation_euler(3))*...
                                RotationAxisAngle([0;1;0],frame_rotation_euler(2))*...
                                RotationAxisAngle([1;0;0],frame_rotation_euler(1));
                            frame_displacement = joint_axis * joint_value;
                            frames_robot(:,:,index_arm) = [frame_rotation_matrix frame_translation + frame_rotation_matrix * frame_displacement; 0 0 0 1];
                        end
                    end
                end
                % Transform the homonegeous matrices to world frame
                for index_arm = 2:length(robot_kinematics)
                    frames_robot(:,:,index_arm) = frames_robot(:,:,robot_kinematics(index_arm).parent_) * frames_robot(:,:,index_arm);
                end
            end
        end
    end
end
