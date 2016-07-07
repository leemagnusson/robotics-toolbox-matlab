%% Robot class definition
% Copyright Verb Surgical Inc.
% Robot class is a class we generate to take the input from:
% link_input and joint_input from URDF file: '****.URDF'
%-----------------------------------------------------------------------
% Properties:
% name_: name of the link and joint pair
% joint_value_: current joint value; automatically set after CalculateFK
% stl_name_: name of the stl
% mass_: mass of the link
% center_of_mass_: center of mass of the link
% inertia_matrix_: inertia matrix of the link
% color_: color of the link
% parent_: parent link/joint pair
% joint_limit_: joint limit
% rpy_: roll pitch yaw angles in fixed frame representation
% xyz_: translation from parent
% joint_axis_: active rotation joint axis
% joint_type_: joint type: "fixed", "revolute" or "prismatic"
% frames_: frames for all links after FK
% frames_in_parent_: frames defined in local frame for all links after FK
% joint_torque_: 11 by 1 joint torque after Inverse dynamics
% coupling_matrix_: joint coupling matrix
% transformation_base_: 4 by 4 homogeneous transformation of the base
%-----------------------------------------------------------------------
% Methods:
% RobotClass.CalculateFK: calculate forward kinematics, type "help
% RobotClass.CalculateFK" for details
% RobotClass.InverseKinematics: calculate inverse kinematics, type "help
% RobotClass.InverseKinematics" for details
% ----------------------------------------------------------------------
% Useage:
% urdf_input = URDF('filename.urdf');
% urdf_link_input = urdf_input.links;
% urdf_joint_input = urdf_input.joints;
% robot_object = RobotClass(urdf_link_input,urdf_joint_input);
% robot_object.transformation_base_ = eye(4);
% q = zeros(11,1); % 11 by 1 joint values
% robot_object.CalculateFK(q)
% gravity_constant = [0;0;9.81];
% joint_torque = robot_object.InverseDynamics(gravity_constant,zeros(11,1),zeros(11,1),'gravity')
% -------------------------------------------------------------------
%%

classdef RobotClass < handle
    
    properties (Access = private)
        parent_ % parent link/joint pair
        rpy_ % roll pitch yaw angles
        xyz_ % translation from parent
        joint_axis_ % active rotation joint axis
        joint_type_ % link type
        coupling_matrix_ % joint coupling matrix
    end
    
    properties (Access = public)
        name_ % name of the link and joint pair
        joint_value_ % current joint value
        stl_name_ % name of the stl
        joint_limit_ % joint limit
        mass_ % mass of the link
        center_of_mass_ % center of mass of the link
        inertia_matrix_ % inertia matrix of the link
        color_ % color of the link
        frames_ % frames for all links after FK
        frames_in_parent_ % frames defined in local frame for all links after FK
        joint_torque_ % 11 by 1 joint torque
        transformation_base_ % 4 by 4 homogeneous transformation of the base
        robot_link_hgtransform_handle_ % handle to robot link stl model
        frame_vertex_ % vertex to draw 3d coordinate system
        robot_frame_hgtransform_handle_ % handle to coordinate system of links
    end
    
    methods
        
        function robot_object = RobotClass(urdf_link_input,urdf_joint_input)
            
            % load the parent number and joint number.
            load('parent_number.mat');
            load('coupling_matrix.mat');
            robot_object.coupling_matrix_ = coupling_matrix;
            if nargin == 2
                for index_link = 1:length(urdf_link_input)
                    % set parent and joint number from the mat file
                    joint_number_index = parent_number(2,index_link);
                    robot_object.parent_(index_link) = parent_number(1,index_link);
                    % set the color if link input has 'visual' field
                    if ~isempty(urdf_link_input{index_link}.visual)
                        robot_object.color_(:,index_link) = urdf_link_input{index_link}.visual.material.color.rgba';
                    else
                        robot_object.color_(:,index_link) = [1; 1; 1; 1];
                    end
                    % set the name following the link name
                    robot_object.name_{index_link} = urdf_link_input{index_link}.name;
                    % Define joint limits
                    if index_link==1
                        % base link is initialized differently
                        robot_object.joint_limit_(:,index_link) = [0;0];
                        robot_object.xyz_(:,index_link) = [0;0;0];
                        robot_object.rpy_(:,index_link) = [0;0;0];
                        robot_object.joint_axis_(:,index_link) = [0;0;1];
                        robot_object.joint_type_{index_link} = 'fixed';
                    else
                        % detect fixed links and set link type andd active
                        % joint axis
                        if strcmp(urdf_joint_input{joint_number_index}.type, 'fixed') || ~isempty(strfind(urdf_joint_input{joint_number_index}.name,'jaw'))
                            robot_object.joint_type_{index_link} = 'fixed';
                            robot_object.joint_axis_(:,index_link) = [0;0;1];
                        else
                            robot_object.joint_type_{index_link} = urdf_joint_input{joint_number_index}.type;
                            robot_object.joint_axis_(:,index_link) = urdf_joint_input{joint_number_index}.axis.xyz';
                        end
                        
                        % set the xyz translation and rpy euler angles
                        robot_object.xyz_(:,index_link) = urdf_joint_input{joint_number_index}.origin.xyz';
                        robot_object.rpy_(:,index_link) = urdf_joint_input{joint_number_index}.origin.rpy';
                        % get joint limits
                        if ~isempty(urdf_joint_input{joint_number_index}.limit)
                            robot_object.joint_limit_(:,index_link) = [urdf_joint_input{joint_number_index}.limit.lower; urdf_joint_input{joint_number_index}.limit.upper];
                        else
                            robot_object.joint_limit_(:,index_link) = [0;0];
                        end
                    end
                    % set stl file name
                    robot_object.stl_name_{index_link} = strcat(urdf_link_input{index_link}.name,'.stl');
                    % check and set inertial terms for dynamics
                    if ~isempty(urdf_link_input{index_link}.inertial)
                        robot_object.mass_(index_link) = urdf_link_input{index_link}.inertial.mass.value;
                        robot_object.center_of_mass_(:,index_link) = urdf_link_input{index_link}.inertial.origin.xyz;
                        robot_object.inertia_matrix_(:,:,index_link) =...
                            [urdf_link_input{index_link}.inertial.inertia.ixx urdf_link_input{index_link}.inertial.inertia.ixy urdf_link_input{index_link}.inertial.inertia.ixz;...
                            urdf_link_input{index_link}.inertial.inertia.ixy urdf_link_input{index_link}.inertial.inertia.iyy urdf_link_input{index_link}.inertial.inertia.iyz;...
                            urdf_link_input{index_link}.inertial.inertia.ixz urdf_link_input{index_link}.inertial.inertia.iyz urdf_link_input{index_link}.inertial.inertia.izz];
                    else
                        robot_object.mass_(index_link) = 0;
                        robot_object.center_of_mass_(:,index_link) = [0;0;0];
                        robot_object.inertia_matrix_(:,:,index_link) = zeros(3,3);
                    end
                end
                robot_object.transformation_base_ = eye(4);
                % intialize the hgtransform handle
                robot_object.robot_link_hgtransform_handle_ = zeros(length(urdf_link_input), 1);
                robot_object.frame_vertex_ = load('frame_3d.mat');
                robot_object.robot_frame_hgtransform_handle_ = zeros(length(urdf_link_input), 3);
            else
                error('Wrong input argument! Check "help RobotClass" for more information');
            end
            
        end
        
        function CalculateFK(robot_object,q)
            % this is the function to calculate the FK of the robot
            % robot_object = RobotClass(urdf_link_input,urdf_joint_input)
            % robot_object.CalculateFK(q_rcm,robot_base_transform)
            % robot_object.frames_: return 4 by 4 by 18 frames for all the
            % links defined in the world frame
            % robot_object.frames_in_parent_: return 4 by 4 by 18 frames
            % for all the links defined in the parent frame
            % q: 11 by 1 joint values
            % robot_base_transform: 4 by 4 homogeneous transformation of
            % the base of the robot
            try
                q_rcm = robot_object.coupling_matrix_*q;
                for index_arm = 1:length(robot_object.name_)
                    % set parent and joint number from the mat file
                    joint_index = robot_object.parent_(index_arm);
                    % Define homogeneous transformation
                    if index_arm==1
                        % base link
                        robot_object.frames_(:,:,index_arm) = robot_object.transformation_base_;
                    else
                        % joint translation and fixed frame roll pitch yaw
                        frame_translation = robot_object.xyz_(:,index_arm);
                        frame_rotation_euler = robot_object.rpy_(:,index_arm);
                        joint_axis = robot_object.joint_axis_(:,index_arm);
                        % active joint value
                        if strcmp(robot_object.joint_type_{index_arm}, 'fixed')
                            joint_value = 0;
                        else
                            joint_value = q_rcm(joint_index);
                        end
                        % homogeneous transformation for revolute and
                        % prismatic joints
                        if ~strcmp(robot_object.joint_type_{index_arm}, 'prismatic')
                            frame_rotation_matrix = RotationAxisAngle([0;0;1],frame_rotation_euler(3))*...
                                RotationAxisAngle([0;1;0],frame_rotation_euler(2))*...
                                RotationAxisAngle([1;0;0],frame_rotation_euler(1))*...
                                RotationAxisAngle(joint_axis,joint_value);
                            robot_object.frames_(:,:,index_arm) = [frame_rotation_matrix frame_translation; 0 0 0 1];
                        else
                            frame_rotation_matrix = RotationAxisAngle([0;0;1],frame_rotation_euler(3))*...
                                RotationAxisAngle([0;1;0],frame_rotation_euler(2))*...
                                RotationAxisAngle([1;0;0],frame_rotation_euler(1));
                            frame_displacement = joint_axis * joint_value;
                            robot_object.frames_(:,:,index_arm) =...
                                [frame_rotation_matrix frame_translation + frame_rotation_matrix * frame_displacement; 0 0 0 1];
                        end
                    end
                end
                robot_object.frames_in_parent_ = robot_object.frames_;
                % Transform the homonegeous matrices to world frame
                for index_arm = 2:length(robot_object.name_)
                    robot_object.frames_(:,:,index_arm) = robot_object.frames_(:,:,robot_object.parent_(index_arm)) * robot_object.frames_(:,:,index_arm);
                end
                robot_object.joint_value_ = q;
            catch
                robot_object.frames_in_parent_ = zeros(4,4,length(robot_object.name_));
                robot_object.frames_ = zeros(4,4,length(robot_object.name_));
            end
        end
        
        function joint_torque = InverseDynamics(robot_object,gravity_constant,q_dot,q_double_dot,mode)
            % Calculate joint torque from inverse dynamics
            % robot_object = RobotClass(urdf_link_input,urdf_joint_input)
            % robot_object.CalculateFK(q)
            % gravity_constant = [0;0;9.81];
            % q_dot: 11 by 1 joint velocity
            % q_double_dot: 11 by 1 joint acceleration
            % mode: 'gravity'; 'dynamic'; 'combined';
            % joint_torque = robot_object.InverseDynamics(gravity_constant,q_dot,q_double_dot,mode)
            % joint_torque: 11 by 1 active joint torque
            try
                q_rcm_dot = robot_object.coupling_matrix_ * q_dot;
                q_rcm_double_dot = robot_object.coupling_matrix_ * q_double_dot;
                num_active_joints = length(q_rcm_dot);
                mass = robot_object.mass_(2:num_active_joints+1);
                center_of_mass = robot_object.center_of_mass_(:,2:num_active_joints+1);
                inertia_matrix = robot_object.inertia_matrix_(:,:,2:num_active_joints+1);
                %             for index_set = 10:13
                %                 mass(index_set) = 0;
                %                 inertia_matrix(:,:,index_set) = zeros(3,3);
                %             end
                if isempty(robot_object.frames_) == 1
                    error('FK first! robot_object.CalculateFK(q_rcm,robot_base_transform)');
                else
                    switch mode
                        case 'gravity'
                            q_rcm_dot = zeros(num_active_joints,1);
                            q_rcm_double_dot = zeros(num_active_joints,1);
                        case 'dynamic'
                            gravity_constant = zeros(3,1);
                            
                        case 'combined'
                            
                        otherwise
                            q_rcm_dot = zeros(num_active_joints,1);
                            q_rcm_double_dot = zeros(num_active_joints,1);
                    end
                    omega_in_own_0 = zeros(3,1);
                    omega_dot_in_own_0 = zeros(3,1);
                    v_dot_in_own_0 = gravity_constant;
                    omega_in_own = zeros(3,num_active_joints);
                    omega_dot_in_own = zeros(3,num_active_joints);
                    v_dot_in_own = zeros(3,num_active_joints);
                    v_c_dot_in_own = zeros(3,num_active_joints);
                    force_in_own = zeros(3,num_active_joints);
                    moment_in_own = zeros(3,num_active_joints);
                    z_in_own = zeros(3,num_active_joints);
                    for i = 0 : num_active_joints - 1
                        rotation_in_parent = robot_object.frames_in_parent_(1:3,1:3,i+2);
                        translation_in_parent = robot_object.frames_in_parent_(1:3,4,i+2);
                        z_in_own(:,i+1) = robot_object.joint_axis_(:,i+2);
                        if i == 0
                            omega_in_own (:,i+1) = rotation_in_parent' * omega_in_own_0 + q_rcm_dot(i+1) * z_in_own(:,i+1);
                            omega_dot_in_own (:,i+1) = rotation_in_parent' * omega_dot_in_own_0 +...
                                cross(rotation_in_parent' * omega_in_own_0,q_rcm_dot(i+1) * z_in_own(:,i+1)) + q_rcm_double_dot(i+1) * z_in_own(:,i+1);
                            v_dot_in_own (:,i+1) = rotation_in_parent' * (cross(omega_dot_in_own_0,translation_in_parent) +...
                                cross(omega_in_own_0, cross(omega_in_own_0,translation_in_parent)) + v_dot_in_own_0);
                            v_c_dot_in_own (:,i+1) = cross(omega_dot_in_own(:,i+1), center_of_mass(:,i+1)) +...
                                cross(omega_in_own(:,i+1),cross(omega_in_own(:,i+1),center_of_mass(:,i+1))) + v_dot_in_own(:,i+1);
                            force_in_own (:,i+1) = mass(i+1) * v_c_dot_in_own (:,i+1);
                            moment_in_own (:,i+1) = inertia_matrix(:,:,i+1) * omega_dot_in_own (:,i+1) +...
                                cross(omega_in_own(:,i+1),inertia_matrix(:,:,i+1) * omega_in_own(:,i+1));
                        elseif ~isempty(strfind(robot_object.name_{i+2},'_tool_translate_'))
                            omega_in_own (:,i+1) = rotation_in_parent' * omega_in_own(:,i);
                            omega_dot_in_own (:,i+1) = rotation_in_parent' * omega_dot_in_own(:,i);
                            v_dot_in_own (:,i+1) = rotation_in_parent' * (cross(omega_dot_in_own(:,i),translation_in_parent) +...
                                cross(omega_in_own(:,i), cross(omega_in_own(:,i),translation_in_parent)) + v_dot_in_own(:,i)) +...
                                rotation_in_parent' * (q_rcm_double_dot(i+1) * z_in_own(:,i+1)) + 2 * cross(q_rcm_dot(i+1) * omega_in_own(:,i+1), rotation_in_parent' * z_in_own(:,i+1));
                            v_c_dot_in_own (:,i+1) = cross(omega_dot_in_own(:,i+1), center_of_mass(:,i+1)) +...
                                cross(omega_in_own(:,i+1),cross(omega_in_own(:,i+1),center_of_mass(:,i+1))) + v_dot_in_own(:,i+1);
                            force_in_own (:,i+1) = mass(i+1) * v_c_dot_in_own (:,i+1);
                            moment_in_own (:,i+1) = inertia_matrix(:,:,i+1) * omega_dot_in_own (:,i+1) +...
                                cross(omega_in_own(:,i+1),inertia_matrix(:,:,i+1) * omega_in_own(:,i+1));
                        else
                            omega_in_own (:,i+1) = rotation_in_parent' * omega_in_own(:,i) + q_rcm_dot(i+1) * z_in_own(:,i+1);
                            omega_dot_in_own (:,i+1) = rotation_in_parent' * omega_dot_in_own(:,i) +...
                                cross(rotation_in_parent' * omega_in_own(:,i),q_rcm_dot(i+1) * z_in_own(:,i+1)) + q_rcm_double_dot(i+1) * z_in_own(:,i+1);
                            v_dot_in_own (:,i+1) = rotation_in_parent' * (cross(omega_dot_in_own(:,i),translation_in_parent) +...
                                cross(omega_in_own(:,i), cross(omega_in_own(:,i),translation_in_parent)) + v_dot_in_own(:,i));
                            v_c_dot_in_own (:,i+1) = cross(omega_dot_in_own(:,i+1), center_of_mass(:,i+1)) +...
                                cross(omega_in_own(:,i+1),cross(omega_in_own(:,i+1),center_of_mass(:,i+1))) + v_dot_in_own(:,i+1);
                            force_in_own (:,i+1) = mass(i+1) * v_c_dot_in_own (:,i+1);
                            moment_in_own (:,i+1) = inertia_matrix(:,:,i+1) * omega_dot_in_own (:,i+1) +...
                                cross(omega_in_own(:,i+1),inertia_matrix(:,:,i+1) * omega_in_own(:,i+1));
                            
                        end
                    end
                    joint_force_in_own = zeros(3,num_active_joints);
                    joint_moment_in_own = zeros(3,num_active_joints);
                    torque_rcm = zeros(num_active_joints,1);
                    for i = num_active_joints : -1 : 1
                        rotation_in_parent = robot_object.frames_in_parent_(1:3,1:3,i+2);
                        translation_in_parent = robot_object.frames_in_parent_(1:3,4,i+2);
                        z_in_own(:,i) = robot_object.joint_axis_(:,i+1);
                        if i == num_active_joints
                            joint_force_in_own (:,i) = force_in_own(:,i);
                            joint_moment_in_own (:,i) = moment_in_own(:,i) + cross( center_of_mass(:,i), force_in_own(:,i));
                            torque_rcm(i) = dot(joint_moment_in_own(:,i),z_in_own(:,i));
                        elseif ~isempty(strfind(robot_object.name_{i+1},'_tool_translate_'))
                            joint_force_in_own (:,i) = rotation_in_parent * joint_force_in_own(:,i+1) +force_in_own(:,i);
                            joint_moment_in_own (:,i) = moment_in_own(:,i) + rotation_in_parent * joint_moment_in_own(:,i+1) +...
                                cross( center_of_mass(:,i), force_in_own(:,i)) + cross(translation_in_parent, rotation_in_parent * joint_force_in_own(:,i+1));
                            torque_rcm(i) = dot(joint_force_in_own(:,i),z_in_own(:,i));
                        else
                            joint_force_in_own (:,i) = rotation_in_parent * joint_force_in_own(:,i+1) +force_in_own(:,i);
                            joint_moment_in_own (:,i) = moment_in_own(:,i) + rotation_in_parent * joint_moment_in_own(:,i+1) +...
                                cross( center_of_mass(:,i), force_in_own(:,i)) + cross(translation_in_parent, rotation_in_parent * joint_force_in_own(:,i+1));
                            torque_rcm(i) = dot(joint_moment_in_own(:,i),z_in_own(:,i));
                        end
                    end
                    robot_object.joint_torque_ = -1 * robot_object.coupling_matrix_' * torque_rcm;
                    joint_torque = robot_object.joint_torque_;
                end
            catch
                joint_torque = zeros(11,1);
                robot_object.joint_torque_ = zeros(11,1);
            end
        end
        
        function [jacobian_spherical,jacobian_cartesian,jacobian_all] = CalculateJacobianAll(robot_object)
            % Jacobian calculation
            % function to export Jacobian
            % the jacobian here are all analytical Jacobians
            % [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll
            % jacobian_spherical: 6 by 6 Jacobian that controls only the spherical arm
            % jacobian_cartesian: 6 by 5 Jacobian that controls only cartesian arm
            % jacobian_all: overall 6 by 11 Jacobian
            % frames: 4 by 4 by n homogeneous transformation matrix with n representing
            % number of arms. This variable is calculated from Robot forward kinematics
            % need to run robot_object.CalculateFK(q)
            %
            % position of related end effector joints
            load('index_joints.mat');
            try
                eef = robot_object.frames_(1:3,4,index_eef); % end effector at distal wrist joint
                % setup position and z axis of each coordinate frame
                for i = 1 : length(robot_object.frames_)
                    p(:,i) = robot_object.frames_(1:3,4,i);
                    z(:,i) = robot_object.frames_(1:3,3,i);
                end
                % J_temp is the Jacobian with 6 by 13 dimension and takes into account all
                % passive/coupled joints
                index = 1;
                for i = 2 : index_eef
                    if i ~= index_tool_translate
                        % revolute joint
                        jacobian_temp(:,index) = [cross(z(:,i),eef - p(:,i));z(:,i)];
                        index = index + 1;
                    else
                        % prismatic joint
                        jacobian_temp(:,index) = [z(:,i);zeros(3,1)];
                        index = index + 1;
                    end
                end
                
                % overall jacobian
                jacobian_all = jacobian_temp * robot_object.coupling_matrix_;
                jacobian_cartesian = jacobian_all(:,1:5);
                jacobian_spherical = jacobian_all(:,6:11);
            catch
                error('Can not calculate Jacobian! try "help RobotClass.CalculateJacobianAll"');
            end
        end
        
        function jacobian_touch = CalculateJacobianGeneral(robot_object,p_touch,num_frame)
            % Jacobian on touch point
            % function to export Jacobian
            % the jacobian here are Jacobian with specific touch point and with
            % selected frame number
            % [jacobian_touch] = robot_object.CalculateJacobianGeneral(p_touch,num_frame)
            % p_touch: position of touch point in world frame
            % num_frame: the index of frame we generate jacobian on.
            % frames: 4 by 4 by n homogeneous transformation matrix with n representing
            % number of arms. This variable is calculated from Robot forward kinematics
            % need to run RobotClass.CalculateFK(q)
            
            % position of related end effector joints
            load('index_joints.mat');
            try
                z=zeros(3,length(robot_object.frames_));
                p=zeros(3,length(robot_object.frames_));
                % setup position and z axis of each coordinate frame
                for index_arm = 1 : length(robot_object.frames_)
                    p(:,index_arm) = robot_object.frames_(1:3,4,index_arm);
                    z(:,index_arm) = robot_object.frames_(1:3,3,index_arm);
                end
                % overall jacobian on touch point with selected frame numbers
                index = 1;
                for index_arm = 2 : num_frame
                    if index_arm ~= index_tool_translate
                        jacobian_temp(:,index) = [cross(z(:,index_arm),p_touch - p(:,index_arm));z(:,index_arm)];
                        index = index + 1;
                    else
                        jacobian_temp(:,index) = [z(:,index_arm);zeros(3,1)];
                        index = index + 1;
                    end
                end
                % calculate the reduced coupling matrix
                if num_frame<=index_pitch_a
                    % up to pitch a joint the active joint numbers = total joint numbers
                    coupling_matrix_reduced = robot_object.coupling_matrix_(1:num_frame-1,1:num_frame-1);
                elseif num_frame==index_pitch_b
                    % active joint number is 1 less than total joint number
                    coupling_matrix_reduced = robot_object.coupling_matrix_(1:num_frame-1,1:num_frame-2);
                else
                    % after pitch c joint, active joint number is always 2 less than total
                    % joint number
                    coupling_matrix_reduced = robot_object.coupling_matrix_(1:num_frame-1,1:num_frame-3);
                end
                
                % overall jacobian
                jacobian_touch = jacobian_temp * coupling_matrix_reduced;
            catch
                error('Can not calculate Jacobian! try "help RobotClass.CalculateJacobianGeneral"');
            end
        end
        
        function jacobian_6dof_rcm = CalculateJacobian6DofRCM(robot_object)
            % Jacobian calculation for 6 DoF rcm point control
            % function to export Jacobian that controls the position of the rcm link
            % and keep the orientation of the rcm link parallel to the spherical roll
            % link. this Jacobian is a special case for CalculateJacobianGeneral
            % [jacobian_6dof_rcm] = robot_object.CalculateJacobian6DofRCM
            % frames: 4 by 4 by n homogeneous transformation matrix with n representing
            % number of arms. This variable is calculated from Robot forward kinematics.
            % need to run robot_object.CalculateFK(q)
            
            % position of related end effector joints
            load('index_joints.mat');
            try
                rcm = robot_object.frames_(1:3,4,index_rcm); % rcm joint
                z_joint=zeros(3,length(robot_object.frames_));
                origin_joint=zeros(3,length(robot_object.frames_));
                % setup position and z axis of each coordinate frame
                for index_arm = 1 : length(robot_object.frames_)
                    origin_joint(:,index_arm) = robot_object.frames_(1:3,4,index_arm);
                    z_joint(:,index_arm) = robot_object.frames_(1:3,3,index_arm);
                end
                % Calculate the Jacocbian
                index = 1;
                for index_arm = 2 : index_car
                    jacobian_6dof_rcm(:,index) = [cross(z_joint(:,index_arm),rcm - origin_joint(:,index_arm));z_joint(:,index_arm)];
                    index = index + 1;
                end
            catch
                error('Can not calculate Jacobian! try "help RobotClass.CalculateJacobianGeneral"');
            end
        end
        
        function q_rcm = ConvertToRcm(robot_object,q)
            % this function converts active joint angles 11 by 1 to coupled joint
            % angles 13 by 1, coupling_matrix is 13 by 11 matrix
            % q_rcm = ConvertToRcm(q)
            q_rcm = robot_object.coupling_matrix_*q;
        end
        
        function q_ref = InverseKinematics(robot_object,q_cur,p_t,rotation_t,ik_mode)
            % Inverse kinematics with speicfied mode
            % [q_ref] = robot_object.InverseKinematics(q_cur,p_t,rotation_t,ik_mode)
            % q_cur: 11 by 1 current joint value
            % p_t and rotation_t: 3 by 1 vector and 3 by 3 rotation matrix
            % of target pose.
            % ik_mode:
            % 'Spherical 6': 6 Dof position and orientation tracking for
            % spherical arm using joint 6~11
            % 'Spherical 3': 3 Dof position tracking for spherical arm
            % using joint 6~11
            % 'Cartesian 6': 6 Dof position and orientation tracking for
            % cartesian arm using joint 1~6
            % 'Cartesian 5': 3 Dof position tracking for cartesian arm
            % using joint 1~5
            % 'Repositioning eef pos': repositioning using all 11 joints
            % but specify only eef linear velocity and hold rcm position
            % stable.
            InitIKParameters;
            q_ref = q_cur;
            load('index_joints.mat');
            % iteration_steps is set to 1000 because this library is not for real-time
            % control. For real-time control the iteration_steps should be re-defined.
            iteration_steps = 0;
            switch ik_mode
                case 'Spherical 6'
                    robot_object.CalculateFK(q_ref);
                    p_cur = robot_object.frames_(1:3,4,index_eef);
                    rotation_cur = robot_object.frames_(1:3,1:3,index_eef);
                    p_err = p_t - p_cur;
                    rotation_err = rotation_t * rotation_cur';
                    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
                    iteration_steps = 0;
                    while (((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation)) && iteration_steps<=1000)
                        iteration_steps = iteration_steps + 1;
                        robot_object.CalculateFK(q_ref);
                        p_cur = robot_object.frames_(1:3,4,index_eef);
                        rotation_cur = robot_object.frames_(1:3,1:3,index_eef);
                        % compute twist
                        [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,rotation_t,rotation_cur);
                        % get Jacobian
                        [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
                        q_dot = pinv(jacobian_spherical)*twist;
                        q_dot_all = [0;0;0;0;0;q_dot];
                        % update q
                        q_ref = q_ref + q_dot_all *dt;
                    end
                case 'Spherical 3'
                    robot_object.CalculateFK(q_ref);
                    p_cur = robot_object.frames_(1:3,4,index_eef);
                    p_err = p_t - p_cur;
                    iteration_steps = 0;
                    while (((norm(p_err) > eps_translation)) && iteraion_steps <= 1000)
                        iteration_steps = iteration_steps + 1;
                        robot_object.CalculateFK(q_ref);
                        p_cur = robot_object.frames_(1:3,4,index_eef);
                        rotation_cur = robot_object.frames_(1:3,1:3,index_eef);
                        % compute twist
                        [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,rotation_t,rotation_cur);
                        % get Jacobian
                        [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
                        q_dot = pinv(jacobian_spherical(1:3,:))*twist(1:3);
                        q_dot_all = [0;0;0;0;0;q_dot];
                        % update q
                        q_ref = q_ref + q_dot_all *dt;
                    end
                case 'Cartesian 6'
                    robot_object.CalculateFK(q_ref);
                    p_cur = robot_object.frames_(1:3,4,index_rcm);
                    rotation_cur = robot_object.frames_(1:3,1:3,index_car);
                    p_err = p_t - p_cur;
                    rotation_err = rotation_t * rotation_cur';
                    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
                    iteration_steps = 0;
                    while (((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation)) && iteraion_steps <= 1000)
                        iteration_steps = iteration_steps + 1;
                        robot_object.CalculateFK(q_ref);
                        p_cur = robot_object.frames_(1:3,4,index_rcm);
                        rotation_cur = robot_object.frames_(1:3,1:3,index_car);
                        [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,rotation_t,rotation_cur);
                        jacobian_6dof = robot_object.CalculateJacobian6DofRCM;
                        q_dot = pinv(jacobian_6dof)*twist;
                        q_dot_all = [q_dot;0;0;0;0;0];
                        q_ref = q_ref + q_dot_all *dt;
                    end
                    
                case 'Cartesian 3'
                    robot_object.CalculateFK(q_ref);
                    p_cur = robot_object.frames_(1:3,4,index_rcm);
                    p_err = p_t - p_cur;
                    iteration_steps = 0;
                    while (((norm(p_err) > eps_translation)) && iteraion_steps <= 1000)
                        iteration_steps = iteration_steps + 1;
                        robot_object.CalculateFK(q_ref);
                        p_cur = robot_object.frames_(1:3,4,index_rcm);
                        rotation_cur = robot_object.frames_(1:3,1:3,index_car);
                        [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,rotation_t,rotation_cur);
                        [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
                        q_dot = pinv(jacobian_cartesian(1:3,:))*twist(1:3);
                        q_dot_all = [q_dot;0;0;0;0;0;0];
                        q_ref = q_ref + q_dot_all *dt;
                    end
                    
                case 'Repositioning eef pos'
                    robot_object.CalculateFK(q_ref);
                    p_cur = robot_object.frames_(1:3,4,index_rcm);
                    p_err = p_t - p_cur;
                    iteration_steps = 0;
                    while (((norm(p_err) > eps_translation)) && iteraion_steps <= 1000)
                        iteration_steps = iteration_steps + 1;
                        robot_object.CalculateFK(q_ref);
                        p_cur = robot_object.frames_(1:3,4,index_eef);
                        rotation_cur = robot_object.frames_(1:3,1:3,index_eef);
                        [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,rotation_t,rotation_cur);
                        
                        [jacobian_rcm,jacobian_car,jacobian_all] = robot_object.CalculateJacobianAll;
                        jacobian_car_6DoF = robot_object.CalculateJacobian6DofRCM;
                        jacobian_repositioning = [jacobian_all(1:3,:);jacobian_car_6DoF(1:3,:) zeros(3,5)];
                        
                        q_dot_all = pinv(jacobian_repositioning)*[twist(1:3);0;0;0];
                        
                        % update q
                        q_ref = q_ref + q_dot_all *dt;
                        
                    end
                    
                otherwise
                    error('No such IK mode!')
            end
        end
        
        function DrawRobot(robot_object, vertex_arm_origin, frame_to_display, frame_scale, alpha_value)
            % Draw one arm of the robot using hgtransform to speed up stl
            % rendering.
            % robot_object.DrawRobot(vertex_arm_origin,plot_frame,axis_length)
            % This function draws one arm of the robot
            % needs at least 1 argument: vertex_arm_origin
            % 2nd and 3rd arguments are frame names and axis length
            % plot_frame = [1 2 3]; % draw joint frame 0 1 and 2
            % axis_length = 0.1; % frame axis length 0.1
            
            if nargin > 1
                if(isempty(robot_object.robot_link_hgtransform_handle_))
                    robot_object.robot_link_hgtransform_handle_ = zeros(length(robot_object.frames_), 1);
                    robot_object.frame_vertex_ = load('frame_3d.mat');
                    robot_object.robot_frame_hgtransform_handle_ = zeros(length(robot_object.frames_), 3);
                end
                % default to hiding the reference frames
                if(~exist('frame_to_display', 'var'))
                    frame_to_display = [];
                end
                for i = 1:length(robot_object.frames_)
                    rotation = robot_object.frames_(1:3,1:3,i);
                    translation = robot_object.frames_(1:3,4,i);
                    
                    % check to display frames
                    if(isempty(robot_object.robot_frame_hgtransform_handle_(i, :)) || ~all(robot_object.robot_frame_hgtransform_handle_(i, :)))
                        % create handle for frame display
                        robot_object.robot_frame_hgtransform_handle_(i, :) = hgtransform('Matrix', robot_object.frames_(:, :, i));
                        
                        % check to display the coordinate systems
                        if(nargin > 3)
                            if(~exist('frame_scale', 'var'))
                                frame_scale = 0.6; % default frame scale
                            end
                            if (~isempty(find(frame_to_display== i, 1)))
                                PlotStl(robot_object.frame_vertex_.arrow_vertex_x', ...
                                    [1,0,0,1], robot_object.robot_frame_hgtransform_handle_(i, 1), frame_scale);
                                PlotStl(robot_object.frame_vertex_.arrow_vertex_y', ...
                                    [0,1,0,1], robot_object.robot_frame_hgtransform_handle_(i, 2), frame_scale);
                                PlotStl(robot_object.frame_vertex_.arrow_vertex_z', ...
                                    [0,0,1,1], robot_object.robot_frame_hgtransform_handle_(i, 3), frame_scale);
                            end
                        end
                    else
                        % check to update the coordinate systems
                        if nargin > 3
                            transform_frame = robot_object.frames_(:, :, i);
                            if (~isempty(find(frame_to_display == i, 1)))
                                set(robot_object.robot_frame_hgtransform_handle_(i, :), 'Matrix', transform_frame);
                            end
                        end
                    end
                    % check to display links
                    if isempty(vertex_arm_origin{1,i}) == 0
                        if(~robot_object.robot_link_hgtransform_handle_(i))
                            rgba(:,i) = robot_object.color_(:,i);
                            if(~exist('alpha_value', 'var'))
                                alpha_value = 1;
                            end
                            rgba(4,i) = alpha_value;
                            % create hgtransform and axis handles
                            robot_object.robot_link_hgtransform_handle_(i) = hgtransform('Matrix', robot_object.frames_(:, :, i));
                            PlotStl(vertex_arm_origin(:, i), rgba(:,i), robot_object.robot_link_hgtransform_handle_(i));
                        else
                            transform_link = robot_object.frames_(:, :, i);
                            if(robot_object.robot_link_hgtransform_handle_(i) ~= 0)
                                set(robot_object.robot_link_hgtransform_handle_(i), 'Matrix', transform_link);
                            end
                        end
                    end
                end
            else
                msgbox('wrong input number!')
            end
            
        end
        
        function DrawRobotGUI(robot_object,vertex_arm_origin,axes_handle,plot_frame,axis_length)
            % Draw one arm of the robot in GUI
            % robot_object.DrawRobot(vertex_arm_origin,axex_handle,plot_frame,axis_length)
            % This function draws one arm of the robot
            % needs at least 2 arguments: vertex_arm_origin, axes_handle
            % from GUI
            % 3rd and 4th arguments are frame names and axis length
            % plot_frame = [1 2 3]; % draw joint frame 0 1 and 2
            % axis_length = 0.1; % frame axis length 0.1
            if nargin > 2
                for i = 1:length(robot_object.frames_)
                    rotation = robot_object.frames_(1:3,1:3,i);
                    translation = robot_object.frames_(1:3,4,i);
                    if isempty(vertex_arm_origin{1,i}) == 0
                        vertex_arm_transformed = transformSTL(vertex_arm_origin(:,i),rotation,translation);
                        rgba = robot_object.color_(:,i);
                        PlotStl(vertex_arm_transformed,rgba,axes_handle)
                        hold on
                    end
                    if nargin > 4
                        if ismember(i,plot_frame)
                            DrawCoordinateSystem([axis_length axis_length axis_length],rotation,translation,'rgb',num2str(i-1))
                            hold on
                        end
                    end
                end
            else
                msgbox('wrong input number!')
            end
            
        end
        
        function DrawRobotNoTool(robot_object,vertex_arm_origin,plot_frame,axis_length)
            % this function draws the arm without tool
            % robot_object.DrawRobotNoTool(vertex_arm_origin,plot_frame,axis_length)
            % This function draws one arm of the robot
            % needs at least 1 argument: vertex_arm_origin
            % 2nd and 3rd arguments are frame names and axis length
            % plot_frame = [1 2 3]; % draw joint frame 0 1 and 2
            % axis_length = 0.1; % frame axis length 0.1
            load('index_joints.mat')
            if nargin > 1
                for index_arm = [1:index_pitch_c index_rcm]
                    rotation = robot_object.frames_(1:3,1:3,index_arm);
                    translation = robot_object.frames_(1:3,4,index_arm);
                    if isempty(vertex_arm_origin{1,index_arm}) == 0
                        vertex_arm_transformed = transformSTL(vertex_arm_origin(:,index_arm),rotation,translation);
                        rgba = robot_object.color_(:,index_arm);
                        PlotStl(vertex_arm_transformed,rgba)
                        hold on
                    end
                    if nargin > 3
                        if ismember(index_arm,plot_frame)
                            DrawCoordinateSystem([axis_length axis_length axis_length],rotation,d,'rgb',num2str(index_arm-1))
                            hold on
                        end
                    end
                end
            else
                msgbox('wrong input number!')
            end
            
        end
        
    end
end
