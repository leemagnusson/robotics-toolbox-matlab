classdef InverseKinematicsClass < handle
    % constant properties
    properties(Constant)
        IK_EPSILON = 1.0e-3;
        IK_SMALL_EPSILON = 1.0e-6;
        IK_TINY_EPSILON = 1.0e-9;
        IK_CONDITION = 1.0e2; % condition number threshold for SVD inverse
        IK_SOLVER_RATE = 400; % inverse kinematic solver running rate(Hz)
        MAX_ITERATION_SOR = 50; % maximum number of iterations for Gauss Seidel SOR
        MAX_ITERATION_IK = 10; % maximum number of iterations for close loop IK solver
        SOR_FACTOR = 1.5; % weighting factor for SOR
        METER_TO_DECIMETER = 10; % convert m to decimeter
        METER_TO_CENTIMETER = 100; % convert m to cm
        METER_TO_MILLIMETER = 1000;% convert m to mm
        ZERO_TO_MAX_VELOCITY_TIME = 0.05; % time from zero to maximum velocity(seconds)
        JOINT_VELOCITY_MAX = [2.0, 2.0 ,2.0, 2.0, 2.0, 6.0, 6.0, 0.15, 2*pi, 2*pi, 2*pi]; % all in (rad/s) except J8(m/s)        
        % minimum joint angle limits, all in (rad) except J8 (m)
        JOINT_ANGLE_LOWER_LIMITS = [-3.6477,  -pi/2, -0.4189, -2.3562, -1.1694, ...
            -4.7124, -1.0154, -0.167, -7.85398, -pi/2, -pi/2];
        % maximum joint angle limits, all in (rad) except J8 (m)
        JOINT_ANGLE_UPPER_LIMITS = [0.1396,  pi/2,  2.7227,  2.3562,  1.9350,  ...
            4.7124,  1.4069,  0.092,  7.85398,  pi/2,  pi/2];        
        SPHERICAL_ARM_DOF = 6; % number of degree of freedom for spherical arm
        SPHERICAL_ARM_BASE_JOINT_INDEX = 6; % base joint index number for the spherical arm
        SPHERICAL_ARM_PRISMATIC_JOINT_INDEX = 3; % the relative joint index of prismatic joint of the spherical arm
        MAXIMUM_LINEAR_VELOCITY = 0.12;% m/s
        MAXIMUM_ANGULAR_VELOCITY = pi;  % rad/s
        JOINT_ANGLE_SAFETY_MARGIN = 0.1; % rad
        JOINT_DISTANCE_SAFETY_MARGIN = 0.01; % m
        WRIST_LINK_INDEX = 13; % index of the wrist link 
    end
    % private properties
    properties(Access = private)
        joint_velocities_prev_;
        robot_arm_dof_;
        coordinate_system_handle_;
    end
    % piublic properties
    properties(Access = public)
        robot_object_;
        length_unit_scale_;
        tracking_type_; % 'absolute' vs 'relative'  
        tolerance_;
        joint_acceleration_max_;
        solver_type_;
        vertex_arm_origin_;
        spherical_arm_joint_limit_upper_;
        spherical_arm_joint_limit_lower_;
        spherical_arm_joint_velocity_max_;
        spherical_arm_joint_acc_max_;
        linear_velocity_task_space_max_;
        spherical_arm_joint_margin_;
        debug_info_on_;
    end

    %public constructors
    methods(Access = public)
        function obj = InverseKinematicsClass(varargin)
            switch nargin
                case 0
                    % default constructor of V1.5 arm
                    % First row of parent number is for link in URDF to tell which parent link
                    % it is attached to. Second row is which joint number defines the
                    % transformation of this link. For each new URDF with different links and
                    % joints in order, this array will change.
                    parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;...
                        0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17]; %#ok<NASGU>
                    save(fullfile(pwd,'../../lib/robot/parent_number.mat'),'parent_number'); % save parent and joint number for current URDF
                    % Coupling matrix: The coupling matrix transform the 11 by 1 joint angles
                    % to 13 by 1 joint angles. The 11 by 1 joint angles are active joint angles
                    % except for the jaws. The 13 by 1 joint angles add pitch b and pitch c
                    % joints behind the pitch a joint. A is 13 by 11 with diagnal ones except
                    % for the coupling components on lines 7 8 9 for pitch a pitch b and pitch
                    % c joints. q_rcm = coupling_matrix * q;
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
                        0 0 0 0 0 0 0 0 0 0 1]; %#ok<NASGU>
                    save(fullfile(pwd, '../../lib/robot/coupling_matrix.mat'),'coupling_matrix'); % save coupling matrix
                    
                    % Load URDF link and joint information
                    folder_name = fullfile(pwd, '../../Arm_version_1.5/');
                    name_urdf_file = 'V1.5_Arm_URDF.URDF';
                    name_stl_folder = strcat(folder_name,'meshes/');
                    name_full_urdf_path= strcat(folder_name, 'robots/',name_urdf_file);
                    urdf_input = URDFParser(name_full_urdf_path);
                    urdf_link_input = urdf_input.links;
                    urdf_joint_input = urdf_input.joints;
                    urdf_joint_sequence = urdf_input.jseq; %#ok<NASGU>
                    vertex_patient_body = stl2matlab(strcat(name_stl_folder,'Ventral_Hernia_Body.STL'));
                    scale_mm_to_m = 1/1000;
                    vertex_patient_body{1} = vertex_patient_body{1} * scale_mm_to_m;
                    vertex_patient_body{2} = vertex_patient_body{2} * scale_mm_to_m;
                    vertex_patient_body{3} = vertex_patient_body{3} * scale_mm_to_m; %#ok<NASGU>
                    
                    % create a robot object
                    obj.robot_object_ = RobotClass(urdf_link_input,urdf_joint_input);
                    % call the calculate forward kinematic to create all
                    % the frames
                    obj.robot_object_.CalculateFK(zeros(length(obj.robot_object_.link_patch_handles_), 1));
                    
                case 1 % could be an arm object or arm version number
                    if(isa(varargin{1}, 'RobotClass'))
                        obj.robot_object_ = varargin{1};
                    else
                        switch varargin{1}
                            case 'V1.0'
                                % default constructor of V1.5 arm
                                % First row of parent number is for link in URDF to tell which parent link
                                % it is attached to. Second row is which joint number defines the
                                % transformation of this link. For each new URDF with different links and
                                % joints in order, this array will change.
                                parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;...
                                    0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17]; %#ok<NASGU>
                                save('../../lib/robot/parent_number.mat','parent_number'); % save parent and joint number for current URDF
                                % Coupling matrix: The coupling matrix transform the 11 by 1 joint angles
                                % to 13 by 1 joint angles. The 11 by 1 joint angles are active joint angles
                                % except for the jaws. The 13 by 1 joint angles add pitch b and pitch c
                                % joints behind the pitch a joint. A is 13 by 11 with diagnal ones except
                                % for the coupling components on lines 7 8 9 for pitch a pitch b and pitch
                                % c joints. q_rcm = coupling_matrix * q;
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
                                    0 0 0 0 0 0 0 0 0 0 1]; %#ok<NASGU>
                                save('../../lib/robot/coupling_matrix.mat','coupling_matrix'); % save coupling matrix
                                
                                % Load URDF link and joint information
                                folder_name = '../../Arm_version_1.0/';
                                name_urdf_file = 'V1.0_Arm_URDF.URDF';
                                name_stl_folder = strcat(folder_name,'meshes/');
                                name_full_urdf_path= strcat(folder_name, 'robots/',name_urdf_file);
                                urdf_input = URDFParser(name_full_urdf_path);
                                urdf_link_input = urdf_input.links;
                                urdf_joint_input = urdf_input.joints;
                                urdf_joint_sequence = urdf_input.jseq; %#ok<NASGU>
                                vertex_patient_body = stl2matlab(strcat(name_stl_folder,'Ventral_Hernia_Body.STL'));
                                scale_mm_to_m = 1/1000;
                                vertex_patient_body{1} = vertex_patient_body{1} * scale_mm_to_m;
                                vertex_patient_body{2} = vertex_patient_body{2} * scale_mm_to_m;
                                vertex_patient_body{3} = vertex_patient_body{3} * scale_mm_to_m; %#ok<NASGU>
                                
                                % create a robot object
                                obj.robot_object_ = RobotClass(urdf_link_input,urdf_joint_input);
                            case 'V2.0'
                                % TODO:
                                % Create V2.0 robot
                            otherwise
                                error('Invalid input type, expected a single RobotClass object, or an arm version');
                        end
                    end
                otherwise
                    error('Invalid input type, expected a single RobotClass object');                    
            end
            
            % initialize other properties
            obj.length_unit_scale_ = InverseKinematicsClass.METER_TO_DECIMETER * 4;
            obj.tracking_type_ = 'relative'; % 'absolute' vs 'relative'
            obj.tolerance_ = InverseKinematicsClass.IK_SMALL_EPSILON;
            obj.joint_acceleration_max_ = ...
                InverseKinematicsClass.JOINT_VELOCITY_MAX / ...
                InverseKinematicsClass.ZERO_TO_MAX_VELOCITY_TIME / ...
                InverseKinematicsClass.IK_SOLVER_RATE; % rad/s^2 or m/s^2
            obj.solver_type_ = 'SVD';% default solver types
            prismatic_joint_index = InverseKinematicsClass.SPHERICAL_ARM_PRISMATIC_JOINT_INDEX;
            obj.spherical_arm_joint_limit_upper_ = ...
                InverseKinematicsClass.JOINT_ANGLE_UPPER_LIMITS( ...
                InverseKinematicsClass.SPHERICAL_ARM_BASE_JOINT_INDEX:end);
            obj.spherical_arm_joint_limit_lower_ = ...
                InverseKinematicsClass.JOINT_ANGLE_LOWER_LIMITS( ...
                InverseKinematicsClass.SPHERICAL_ARM_BASE_JOINT_INDEX:end);
            obj.spherical_arm_joint_limit_lower_(prismatic_joint_index) = ...
            obj.spherical_arm_joint_limit_lower_(prismatic_joint_index) * ...
                obj.length_unit_scale_;
            obj.spherical_arm_joint_limit_upper_(prismatic_joint_index) = ...
            obj.spherical_arm_joint_limit_upper_(prismatic_joint_index) * ...
                obj.length_unit_scale_;
        
            obj.spherical_arm_joint_velocity_max_ = ...
                InverseKinematicsClass.JOINT_VELOCITY_MAX( ...
                InverseKinematicsClass.SPHERICAL_ARM_BASE_JOINT_INDEX:end);
            
            obj.spherical_arm_joint_velocity_max_(prismatic_joint_index) = ...
                obj.spherical_arm_joint_velocity_max_(prismatic_joint_index) * ...
                obj.length_unit_scale_;
                
            obj.spherical_arm_joint_acc_max_ =  ...
                obj.spherical_arm_joint_velocity_max_ / ...
                InverseKinematicsClass.ZERO_TO_MAX_VELOCITY_TIME;
            obj.spherical_arm_joint_acc_max_(prismatic_joint_index) = ...
                obj.spherical_arm_joint_acc_max_(prismatic_joint_index) *...
                obj.length_unit_scale_;
            
            obj.linear_velocity_task_space_max_ = obj.MAXIMUM_LINEAR_VELOCITY * ...
                obj.length_unit_scale_;
            
            obj.spherical_arm_joint_margin_ = ...
                ones(obj.SPHERICAL_ARM_DOF, 1) * obj.JOINT_ANGLE_SAFETY_MARGIN;
            obj.spherical_arm_joint_margin_(obj.SPHERICAL_ARM_PRISMATIC_JOINT_INDEX) = ...
                obj.JOINT_DISTANCE_SAFETY_MARGIN * obj.length_unit_scale_;
           
            obj.robot_arm_dof_ = length(InverseKinematicsClass.JOINT_ANGLE_UPPER_LIMITS);
            obj.joint_velocities_prev_ = zeros(obj.robot_arm_dof_, 1);
            obj.debug_info_on_ = false;
            
        end
    end
    % other public methods
    methods(Access = public)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % compute inverse kinematics
        function [theta_ik, velocities_ik, twist, pose_current_ik, ...
                ik_iteration_steps_ik] = compute_inverse_kinematics(obj, ...
                xyz_command, rotation_command, dt, theta_current_ik, ...
                theta_start_ik, tip_index)
            % get some constants
            spherical_base_joint_index = ...
                InverseKinematicsClass.SPHERICAL_ARM_BASE_JOINT_INDEX;
            
            % calculate forward kinematics
            obj.robot_object_.CalculateFK(theta_current_ik);
            
            % scale the translation coordinates
            for ii = 1: length(obj.robot_object_.frames_)
                obj.robot_object_.frames_(1:3, 4, ii) = ...
                    obj.robot_object_.frames_(1:3, 4, ii) * obj.length_unit_scale_;
                obj.robot_object_.frames_in_parent_(1:3, 4, ii) = ...
                    obj.robot_object_.frames_in_parent_(1:3, 4, ii) * obj.length_unit_scale_;
            end
            
            % get current pose
            pose_current_ik = obj.robot_object_.frames_(:, :, tip_index);
            
            xyz_current = pose_current_ik(1:3, 4);
            rotation_current = pose_current_ik(1:3, 1:3);
            
            % keep a copy of the initial pose before the ik
            pose_prev = pose_current_ik;
            
            % some initialization
            twist_error_max_ik = inf;
            ik_iteration_steps_ik = 0;
            joint_velocities_ik = zeros(obj.SPHERICAL_ARM_DOF , 1);
            %joint_velocity_intermediate = joint_velocities_ik;
            
            % update the maximum velocity change in dt
            obj.spherical_arm_joint_acc_max_ = ...
                obj.spherical_arm_joint_velocity_max_ / ...
                InverseKinematicsClass.ZERO_TO_MAX_VELOCITY_TIME * dt;
            
            % start iteration
            while (twist_error_max_ik > InverseKinematicsClass.IK_SMALL_EPSILON && ...
                    ik_iteration_steps_ik < InverseKinematicsClass.MAX_ITERATION_IK)
                % update the ik step count
                ik_iteration_steps_ik = ik_iteration_steps_ik + 1;

                if(dt == 0)
                    twist_ik = zeros(6, 1);
                else
                    linear_velocity_ik = (xyz_command- xyz_current) /dt;
                    angular_velocity = ...
                        obj.calculate_angular_velocity_from_rotation_diff(...
                        rotation_current, rotation_command, dt);
                    twist_ik = [linear_velocity_ik; angular_velocity];

                    % scale twist by maximum twist
                    for i = 1: length(twist_ik)
                        max_linear_velocity = max(abs(twist_ik(1:3)));
                        max_angular_velocity = max(abs(twist_ik(4:6)));
                        if(max_linear_velocity > obj.linear_velocity_task_space_max_)
                            linear_scale = obj.linear_velocity_task_space_max_ / max_linear_velocity;
                            twist_ik(1:3) = linear_scale * twist_ik(1:3);
                        end
                        if(max_angular_velocity > InverseKinematicsClass.MAXIMUM_ANGULAR_VELOCITY)
                            angular_scale = InverseKinematicsClass.MAXIMUM_ANGULAR_VELOCITY / max_angular_velocity;
                            twist_ik(4:6) = angular_scale * twist_ik(4:6);
                        end
                    end
                end
                % calculate the maximum twist error
                twist_error_max_ik = max(abs(twist_ik));
                
                % calculate jacobian, the first output is the spherical arm jacobian
                [jacobian, ~, ~ ] = obj.robot_object_.CalculateJacobianAll;
                
                % calcaulte joint velocities
                [joint_velocities_ik_step] = obj.calculate_joint_velocities(jacobian, ...
                    twist_ik, ...
                    obj.joint_velocities_prev_(spherical_base_joint_index : end), ...
                    theta_current_ik, ...
                    joint_velocities_ik);
                % scale the prismatic joint velocity
                joint_velocities_ik_step(3) = joint_velocities_ik_step(3) / obj.length_unit_scale_;
                
                % advance the joint angle change
                theta_current_ik(end - InverseKinematicsClass.SPHERICAL_ARM_DOF + 1: end) = ...
                    theta_current_ik(end - InverseKinematicsClass.SPHERICAL_ARM_DOF + 1: end) +...
                    joint_velocities_ik_step * dt;
                
                % calculate the forward kinematics with current joint values
                obj.robot_object_.CalculateFK(theta_current_ik)
                
                % scale the translation coordinates
                for ii = 1: length(obj.robot_object_.frames_)
                    obj.robot_object_.frames_(1:3, 4, ii) = ...
                        obj.robot_object_.frames_(1:3, 4, ii) * obj.length_unit_scale_;
                    obj.robot_object_.frames_in_parent_(1:3, 4, ii) = ...
                        obj.robot_object_.frames_in_parent_(1:3, 4, ii) * obj.length_unit_scale_;
                end
                
                % recalculate joint velocities from the ik solution
                if(dt ~= 0.0)
                    joint_velocities_ik(end - InverseKinematicsClass.SPHERICAL_ARM_DOF + 1: end) = ...
                        (theta_current_ik(end - InverseKinematicsClass.SPHERICAL_ARM_DOF + 1: end) - ...
                        theta_start_ik(end - InverseKinematicsClass.SPHERICAL_ARM_DOF + 1: end)) / dt;
                end
                
                % get the current pose and rotation
                pose_current_ik = obj.robot_object_.frames_(:, :, tip_index);
                rotation_current = pose_current_ik(1:3, 1:3);
                xyz_current = pose_current_ik(1:3, 4);

            end
            
            % recalculate the total twist            
            linear_velocity_ik = (xyz_current- pose_prev(1:3, 4)) /dt;
            angular_velocity = ...
                obj.calculate_angular_velocity_from_rotation_diff(...
                pose_prev(1:3, 1:3), rotation_current, dt);
            twist = [linear_velocity_ik; angular_velocity];
            
            % assign outputs
            theta_ik = theta_current_ik;
            velocities_ik = joint_velocities_ik;
            obj.joint_velocities_prev_(spherical_base_joint_index : end) = ...
                joint_velocities_ik;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [angular_velocity, linear_velocity, time_stamp_int, ...
                xyz_int, quaternion_int] = ...
                load_data(obj, file_name, robot_index, delta_t)
            [angular_velocity, linear_velocity, time_stamp_int, xyz_int, ...
                quaternion_int] = ...
                obj.load_suture_data(file_name, robot_index, delta_t);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function DrawRobot(obj, axis_range)
            % draw robot
            obj.robot_object_.DrawRobot(obj.vertex_arm_origin_, ...
                14, 0.2 * obj.length_unit_scale_, 1.0);
            % update wrist link color
            set(obj.robot_object_.link_patch_handles_(obj.WRIST_LINK_INDEX), ...
                'facec', [0.1, 0.2, 0.2]);
           
            % zoom in wrist area
            axis(axis_range * obj.length_unit_scale_);
        end

    end
    
    % define some static methods
    methods(Static)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function q_inv = invert_quaternion(q)
            quaternion = [q(1), -q(2), -q(3), -q(4)];
            q_squared_norm = sum(q.^2);
            q_inv = quaternion / q_squared_norm;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function rotation = quaternion_difference_to_rotation(q1, q2)
            quaternion =quaternion_multiplication(invert_quaternion(q1), q2);
            quaternion([1,4]) = quaternion([4,1]);
            rotation = QuaternionToRotation(quaternion);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function rotation = quaternion_to_rotation_wxyz(quaternion)
            % rotation = QuaternionToRotation_wxyz(quaternion)
            % function to convert unit quaternion to rotation matrix
            % input quoternion is in the form of [qw, qx, qy, qz]
            % need to convert to [qw, qx, qy, qz]
            quaternion([1,4]) = quaternion([4,1]);
            rotation = QuaternionToRotation(quaternion);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [q_out] = interpolate_quaternion(q_start, q_end, number_of_intepolations)
            % get intermediate point using slerp, this output includes
            % the starting point
            for ii = 1: number_of_intepolations + 1
                q_out(ii, :) = slerp_quaternion(q_start, q_end, (ii - 1) / number_of_intepolations);
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [x_out] = interpolate_linear_data(x_start, x_end, number_of_interpolations)
            % linear interpolation from starting vector x_start
            step_size = (x_end - x_start) / (number_of_interpolations + 1);
            
            for ii = 1: number_of_interpolations + 1
                x_out(ii, :) = x_start + step_size * ( ii - 1);
            end
        end
        
        
        function [q_out, half_theta] = slerp_quaternion(qa, qb, t)
            % http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
            %
            % Calculate angle between two input quaternions.
            cos_half_theta = dot(qa, qb);
            if (cos_half_theta < 0.0)
                qb = - qb;
                cos_half_theta = -cos_half_theta;
            end
            
            % Calculate temporary values.
            half_theta = acos(cos_half_theta);
            sin_half_theta = sqrt(1.0 - cos_half_theta^2);
            if(sin_half_theta > InverseKinematicsClass.IK_SMALL_EPSILON)
                ratioA = sin((1 - t) * half_theta) / sin_half_theta;
                ratioB = sin(t * half_theta) / sin_half_theta;
            else
                ratioA = 0.5;
                ratioB = 0.5;
            end
            %calculate quaternion.
            q_out = qa * ratioA + qb * ratioB;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function q_half = compute_half_rotation_quaternion(q_in)
            %http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/halfAngle.htm
            % quaternion in the form of [qw, qx, qy, qz]
            q_half = q_in;
            q_half(1) = q_in(1) + 1.0;
            q_half = q_half / norm(q_half);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [euler_angles] = compute_euler_angles_from_rotation_matrix(r)
            euler_angles = zeros(1,3);
            if(abs(r(3,1)) == 1.0 )
                euler_angles(1) = 0.0;
                if( r(3,1) == -1.0 )
                    euler_angles(2) = pi/2;
                    euler_angles(3) = atan2(r(1,2), r(1,3) );
                else
                    euler_angles(2) = -pi/2;
                    euler_angles(3) = atan2(-r(1,2), -r(1,3) );
                end
            else
                euler_angles(1) = -asin(r(3, 1));
                % only care about the small angles
                if(abs(euler_angles(1)) > abs(pi - euler_angles(1)))
                    euler_angles(1) = pi - euler_angles(1);
                end
                cos_theta = cos(euler_angles(1));
                euler_angles(2) = atan2(r(3,2) / cos_theta, r(3,3) / cos_theta);
                euler_angles(3) = atan2(r(2,1) / cos_theta, r(1,1) / cos_theta);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function q_conjugate = get_quaternion_conjugate(q)
            q_conjugate = q;
            q_conjugate(2:4) = -q(2:4);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function euler_angles = compute_euler_angles_from_quaternion(q)
            
            % https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            % assume q is normalized
            %
            
            % check for singularity
            
            euler_angles(1) = atan2(2 * (q(1) * q(2) + q(3) *q(4)), ...
                1 - 2 *(q(2) ^2 + q(3) ^ 2));
            
            euler_angles(2) = asin(2*( q(1) * q(3) - q(4) * q(2)));
            
            euler_angles(3) = atan2(2 * (q(1) * q(4) + q(2) *q(3)), ...
                1 - 2 *(q(3) ^2 + q(4) ^ 2));
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function quaternion = rotation_to_quaternion_wxyz(rotation)
            % quaternion = RotationToQuaternion(rotation)
            % function to convert rotation matrix to quaternion
            % Note: output quaternion in the format of [qw, qx, qy, qz]
            quaternion = RotationToQuaternion(rotation);
            quaternion([1,4]) = quaternion([4,1]);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function n = quaternion_multiplication(q, r)
            % n = QuaternionMultiplication(q, r)
            % function to calculate quaternion multiplication
            % The quaternions have the form of [w, x, y, z]
            
            % q=q0 + i*q1 + j*q2 + k*q3
            % and
            % r= r0 + i*r1 + j*r2 + k*r3
            % The quaternion product has the form of
            % n= q×r = n0 + i*n1 + j*n2 + k*n3
            % n0=(r0q0 ? r1q1 ? r2q2 ? r3q3)
            % n1=(r0q1 + r1q0 ? r2q3 + r3q2)
            % n2=(r0q2 + r1q3 + r2q0 ? r3q1)
            % n3=(r0q3 ? r1q2 + r2q1 + r3q0)
            
            n(1) = r(1) * q(1) - r(2) * q(2) - r(3) * q(3) - r(4) * q(4);
            n(2) = r(1) * q(2) + r(2) * q(1) - r(3) * q(4) + r(4) * q(3);
            n(3) = r(1) * q(3) + r(2) * q(4) + r(3) * q(1) - r(4) * q(2);
            n(4) = r(1) * q(4) - r(2) * q(3) + r(3) * q(2) + r(4) * q(1);
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function angle_delta = calculate_angular_velocity_from_quaternion_rotation(q, dt)
            % Function angle_delta = calculate_anguler_velocity_from_quaternion_rotation(q, dt)
            % calcualte angular displacement from infinitestimal quaternion
            % rotation
            % q = cos(angle/2) + i ( x * sin(angle/2)) + j (y * sin(angle/2)) + k ( z * sin(angle/2))
            
            % normalize the quaternion
            if(q(1) > 1.0)
                d = sqrt(sum(q.^2));
                q = q ./d;
            end
            
            half_angle = acos(q(1));
            
            if(abs(half_angle) < InverseKinematicsClass.IK_SMALL_EPSILON)
                x = q(2);
                y = q(3);
                z = q(4);
            else
                if(half_angle < -InverseKinematicsClass.IK_SMALL_EPSILON)
                    half_angle = -half_angle;
                    sine_half_angle = sqrt(1.0 - cos(half_angle) ^2);
                    x = - q(2) / sine_half_angle;
                    y = - q(3) / sine_half_angle;
                    z = - q(4) / sine_half_angle;
                else
                    sine_half_angle = sqrt(1.0 - cos(half_angle) ^2);
                    x = q(2) / sine_half_angle;
                    y = q(3) / sine_half_angle;
                    z = q(4) / sine_half_angle;
                end
            end
            
            % project the angles to the fixed axis
            angle_delta(1) = 2.0 * half_angle * x;
            angle_delta(2) = 2.0 * half_angle * y;
            angle_delta(3) = 2.0 * half_angle * z;
            angle_delta = angle_delta' / dt;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function omega = calculate_anguler_velocity_from_quaternion(qd, q)
            % Function angle_delta = calculate_anguler_velocity_from_quaternion(qd, q)
            % calcualte angular velocity from infinitestimal quaternion
            % rotation
            % qd = 0.5 * W_body * q
            
            q_matrix = [...
                -q(2), -q(3), -q(4); ...
                q(1),  q(4), -q(3); ...
                -q(4),  q(1),  q(2);...
                q(3), -q(2),  q(1)];
            
            omega = 2 * pinv(q_matrix) * qd;
            %         omega_moving = 2 * pinv(q_matrix) * qd;
            
            %         q_omega(1) = 0;
            %         q_omega(2:4) = omega_moving;
            %
            %         q_conj = get_quaternion_conjugate(q);
            %         q_tmp = quaternion_multiplication(q, q_omega);
            %         q_omega = quaternion_multiplication(q_tmp, q_conj);
            %
            %         omega = q_omega(2:4);
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function omega = calculate_angular_velocity_from_rotation(rotation_cur, rotation_t, dt)
            % angular velocity
            R_err = rotation_t * rotation_cur';
            theta_err = acos((trace(R_err) - 1) / 2);
            vect_err = 1/( 2 * sin(theta_err)) * ...
                [(R_err(3,2) - R_err(2,3));
                (R_err(1,3) - R_err(3,1));
                (R_err(2,1) - R_err(1,2))];
            
            omega = theta_err * vect_err / dt;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function angle_diff = calculate_angle_from_rotation(rotation_cur, rotation_t)
            % angular velocity
            R_err = rotation_t * rotation_cur';
            theta_err = acos((trace(R_err) - 1) / 2);
            vect_err = 1/( 2 * sin(theta_err)) * ...
                [(R_err(3,2) - R_err(2,3));
                (R_err(1,3) - R_err(3,1));
                (R_err(2,1) - R_err(1,2))];
            
            angle_diff = theta_err * vect_err;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function omega = calculate_angular_velocity_from_rotation_diff(rotation_initial, rotation_target, dt)
            % angular velocity
            rotation = rotation_target * rotation_initial';
            theta = acos((trace(rotation) - 1) / 2);
            
            if (abs(sin(theta)) > InverseKinematicsClass.IK_SMALL_EPSILON)
                W = 0.5 * theta / (dt * sin(theta)) * (rotation - rotation');
                omega(1,1) = W(3,2);
                omega(2,1) = W(1,3);
                omega(3,1) = W(2,1);
            else
                omega = zeros(3,1);
            end
            
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %private member functions
    methods(Access = private)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [joint_velocities] = calculate_joint_velocities(obj, ...
                jacobian, twist, joint_velocities_prev, theta_current, ...
                joint_velocity_intermediate)
            
            % define supported solver type
            supported_solver_type = {'Projected Gauss Seidel SOR with Limits', 'SVD', ...
                'SVD Condition', 'Pseudoinverse', ...
                'Damped Least Square', 'Damped Projected Gauss Seidel SOR with Limits', ...
                'Gauss Seidel SOR', 'Damped Gauss Seidel SOR'};
            
            % call solver according to user input
            switch obj.solver_type_
                case 'Projected Gauss Seidel SOR with Limits'
                    % projected gauss seidel with Over Relaxation
                    
                    % compose A and b for gauss seidel solver
                    A = jacobian' * jacobian;
                    b = jacobian' * twist;
                    
                    % pivoting column
                    for ii = 1 : obj.SPHERICAL_ARM_DOF - 1
                        [~, col_max_index] = max(abs(A(ii:end, ii)));
                        if(col_max_index ~= 1)
                            A([col_max_index + ii - 1, ii], :) = ...
                                A([ii, col_max_index + ii - 1], :);
                            b([col_max_index + ii - 1, ii]) = ...
                                b([ii, col_max_index + ii - 1]);
                        end
                    end
                    
                    % call gauss seidel solver
                    [joint_velocities, residual_err, iteration_count, is_converged, ...
                        iteration_status] = gauss_seidel_sor(obj, A, b, ...
                        joint_velocities_prev,... 
                        theta_current, joint_velocity_intermediate, true);
                    % check and display debug information
                    if(obj.debug_info_on_)
                        disp([is_converged, iteration_count, residual_err]);
                        disp(iteration_status);
                    end
                    
                case 'SVD'
                    joint_velocities = pinv(jacobian) * twist;
                 
                    
                case 'SVD Condition'
                    % jacobian svd inverse with conditioning
                    [ju, js, jv] = svd(jacobian);
                    
                    max_singular_value = max(max(js));
                    
                    % inverse js
                    js = pinv(js);
                    
                    % check for singularity
                    for ii = 1: min(size(js))
                        if js(ii , ii)/ max_singular_value < ...
                                1/InverseKinematicsClass.IK_CONDITION
                            js(ii , ii) = 0.0;
                        end
                    end
                    
                    joint_velocities = jv * js * ju' * twist;
                    
                case 'Pseudoinverse'
                    % pseudoinverse
                    jtj = jacobian' * pinv(jacobian * jacobian');
                    joint_velocities = jtj * twist;
                    
                case 'Damped Least Square'
                    % damped least square
                    lamda_dls = 0.2;
                    jdls = jacobian * jacobian' + lamda_dls ^ 2 * eye(obj.SPHERICAL_ARM_DOF);
                    joint_velocities = jacobian' * (jdls \ twist);
                    
                case 'Damped Projected Gauss Seidel SOR with Limits'
                    % gauss seidel with lamda
                    lamda_dls = 0.1;
                    A = jacobian' * jacobian + lamda_dls ^ 2 * eye(obj.SPHERICAL_ARM_DOF);
                    b = jacobian' * twist;
                    
                    % call gauss seidel solver
                    [joint_velocities, residual_err, iteration_count, is_converged, ...
                        iteration_status] = gauss_seidel_sor(obj, A, b, ...
                        joint_velocities_prev, ...
                        theta_current, ...
                        joint_velocity_intermediate, true); 
                    
                    % check and display debug information
                    if(obj.debug_info_on_)
                        disp([is_converged, iteration_count, residual_err]);
                        disp(iteration_status);
                    end
                    
                case 'Gauss Seidel SOR'
                    % gauss seidel w/o limits
                    A = jacobian' * jacobian;
                    b = jacobian' * twist;
                    
                    % call gauss seidel solver
                    [joint_velocities, residual_err, iteration_count, is_converged, ...
                        iteration_status] = gauss_seidel_sor(obj, A, b, ...
                        joint_velocities_prev, ...
                        theta_current, ...
                        joint_velocity_intermediate, false);
                    
                    % check and display debug information
                    if(obj.debug_info_on_)
                        disp([is_converged, iteration_count, residual_err]);
                        disp(iteration_status);
                    end
                    
                case 'Damped Gauss Seidel SOR'
                    % gauss seidel w/o limits
                    lamda_dls = 0.01;
                    A = jacobian' * jacobian + lamda_dls ^ 2 * eye(obj.SPHERICAL_ARM_DOF);
                    b = jacobian' * twist;
                    
                    % call gauss seidel solver
                    [joint_velocities, residual_err, iteration_count, is_converged, ...
                        iteration_status] = gauss_seidel_sor(obj, A, b, ...
                        joint_velocities_prev,  ...
                        theta_current, ...
                        joint_velocity_intermediate, false); 
                    
                    % check and display debug information
                    if(obj.debug_info_on_)
                        disp([is_converged, iteration_count, residual_err]);
                        disp(iteration_status);
                    end
                    
                otherwise
                    msg = sprintf('Unsupported solver: ''%s''\nSupported solver types:\n', obj.solver_type_);
                    for ii = 1: length(supported_solver_type)
                        msg = [msg, sprintf('''%s''\n', supported_solver_type{ii})];
                    end
                    error(msg);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [x, residual_err, iteration_count, is_converged, iteration_status] = ...
                gauss_seidel_sor(obj, A, b, x0, joint_angles, ...
                joint_velocity_intermediate, limit_check_enbled)
            % https://en.wikipedia.org/wiki/Gauss-Seidel_method
            % find a closest solution x, such that Ax = b by using gauss seidel method
            % x is bounded by the lower and upper limits and rate of change of x is also
            % limited
            
            % get come constants
            dof = InverseKinematicsClass.SPHERICAL_ARM_DOF;
            sor_factor = InverseKinematicsClass.SOR_FACTOR;
            joint_angle_lower_limits = obj.spherical_arm_joint_limit_lower_;
            joint_angle_upper_limits = obj.spherical_arm_joint_limit_upper_;
            joint_velocity_limits = obj.spherical_arm_joint_velocity_max_;
            joint_acceleration_limits = obj.spherical_arm_joint_acc_max_;
            
            % check A, b compatiblitity
            if length(b) ~= size(A, 2)
                error(...
                    'A and b are not compatible, size of A = %d, size of b = %d', ...
                    size(A), size(b));
            end
           
            % set initial values for some inputs if not defined
            if ~exist('x0', 'var')
                x0 = zeros(dof, 1);
            end          
            
            % first, get the diagonal elements of A
            a_diag = diag(A);
            if ~any(a_diag)
                error('A matrix has zero on the diagonal');
            end
            % pre-scaling
            b = b ./ a_diag;
            for ii = 1: size(A ,1)
                A(ii, :) = A(ii, :) / a_diag(ii);
            end
            
            % initialize a scaled b
            b_scaled = b;
            
            % some initialization
            iteration_count = 1;
            iteration_count_stage = 1;
            iteration_error = inf;
            x = zeros(dof, 1);
            x_prev = x;
            is_converged = false; %#ok<NASGU>
            iteration_status = zeros(3, dof);
            residual_err = inf;
            x_sum = zeros(dof, 1);
            while ((iteration_count < InverseKinematicsClass.MAX_ITERATION_SOR) && ...
                    (iteration_error > InverseKinematicsClass.IK_SMALL_EPSILON))
                
                % intialize scaling factor for every iteration cycle
                scale_factor = inf;                
     
                for ii = 1 : size(A, 1)
                    x_iterate(ii) = 0.0;
                    for jj = 1 : dof
                        if( ii ~= jj)
                            x_iterate(ii) =  x_iterate(ii) + sor_factor * ...
                                A(ii, jj) * x(jj);
                        end
                    end
                    x(ii) = sor_factor * b_scaled(ii) - x_iterate(ii) + ...
                        (1 - sor_factor) * x_prev(ii);

                    if(limit_check_enbled)
                        % get the joint index of the spherical arm
                        joint_index = ...
                            ii + InverseKinematicsClass.SPHERICAL_ARM_BASE_JOINT_INDEX - 1;
                        
                        % calculate accumulated joint velocities
                        x_sum(ii) = joint_velocity_intermediate( ii ) + x(ii);
                        
                        % check for joint angle limits
                        joint_safety_margin = obj.spherical_arm_joint_margin_;
                        if( joint_angles(joint_index) >= ...
                                (joint_angle_upper_limits(ii) - ...
                                joint_safety_margin(ii)) && x(ii) > 0.0)
                            x(ii) = x(ii) / joint_safety_margin(ii) * ...
                                (joint_angle_upper_limits(ii) - joint_angles(joint_index));
                            iteration_status(1, ii) = 1;
                        elseif( joint_angles(joint_index) <= ...
                                (joint_angle_lower_limits(ii) + ...
                                joint_safety_margin(ii)) && x(ii) < 0.0)
                            x(ii) = x(ii) / joint_safety_margin(ii) * ...
                                (joint_angles(joint_index) - joint_angle_lower_limits(ii));
                            iteration_status(1, ii) = -1;                        
                        else
                            iteration_status(1, ii) = 0;
                        end
                        
                        %check for joint velocity upper limits
                        if(joint_velocity_intermediate( ii ) >= joint_velocity_limits(ii))
                            x(ii) = 0.0;
                            iteration_status(2, ii) = 1;
                        else
                            x_sum(ii) = joint_velocity_intermediate( ii ) + x(ii);
                            if( x_sum(ii) > joint_velocity_limits(ii) )
                                scale_iterate = joint_velocity_limits(ii) / x(ii);
                                if(scale_iterate < scale_factor )
                                    scale_factor = scale_iterate;
                                end
                                x(ii) = joint_velocity_limits(ii) - ...
                                    joint_velocity_intermediate(ii);
                                iteration_status(2, ii) = 1;
                            else
                                iteration_status(2, ii) = 0;
                            end
                        end
                    
                        
                        % check for joint velocity lower limits
                        if(joint_velocity_intermediate( ii ) <= -joint_velocity_limits(ii))
                            x(ii) = 0.0;
                            iteration_status(2, ii) = -1;
                        else
                            if( x_sum(ii) < - joint_velocity_limits(ii) )
                                scale_iterate = -joint_velocity_limits(ii) / x(ii);
                                if(scale_iterate < scale_factor )
                                    scale_factor = scale_iterate;
                                end
                                
                                x(ii) = -joint_velocity_limits(ii) - ...
                                    joint_velocity_intermediate(ii);
                                iteration_status(2, ii) = -1;
                            else
                                iteration_status(2, ii) = 0;
                            end
                        end                        
                    end
                end
                % update intermediate x for next step
                x_prev = x;
                
                % calculate iteration error
                error_vector = A * x - b;
                % linear and angular error are calculated seperately
                residual_err = max( norm( error_vector(1:3) ), norm(error_vector(4 : 6)) );
                
                iteration_error = residual_err;
                
                % update iteration count
                if(iteration_count_stage > 15)
                    % check which commands(linear or angular) need to be scaled
                    % by checking the error
                    if isfinite(scale_factor)
                        if scale_factor > 1
                            scale_factor = 1.0;
                        end
                        if(norm( error_vector(1:3) ) > 0.1 )
                            % scale the linear command
                            b_scaled(1:3) = scale_factor * b_scaled(1:3);
                        end
                        if(norm( error_vector(4:6) ) > 0.1 )
                            % scale the angular command
                            b_scaled(4:6) = scale_factor * b_scaled(4:6);
                        end
                    end
                    iteration_count_stage = 1;
                end
                % increment the counter
                iteration_count_stage = iteration_count_stage + 1;
                iteration_count = iteration_count + 1;
            end
            
            %check for accelration limit
            if(limit_check_enbled)
                % ignore the first cycle
                if(~any(x0))
                    for ii = 1 : dof
                        x_sum(ii) = joint_velocity_intermediate( ii ) + x(ii);
                        if( abs(x_sum(ii) - x0(ii)) > joint_acceleration_limits(ii))
                            if x_sum(ii) > x0(ii)
                                x(ii) = x0(ii) + joint_acceleration_limits(ii) - ...
                                    joint_velocity_intermediate(ii);
                                iteration_status(3, ii) = 1;
                            else
                                x(ii) = x0(ii) - joint_acceleration_limits(ii) - ...
                                    joint_velocity_intermediate(ii);
                                iteration_status(3, ii) = -1;
                            end
                        else
                            iteration_status(3, ii) = 0;
                        end
                    end
                end
            end
            
            % check if there is a convergence
            if(iteration_count < InverseKinematicsClass.MAX_ITERATION_SOR)
                is_converged = true;
            else
                is_converged = false;
            end
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [angular_velocity, linear_velocity, time_stamp_int, xyz_int, quaternion_int] = ...
                load_suture_data(obj, file_name, robot_index, delta_t)
            
            fp = fopen(file_name);
            ii = 1;
            while(~feof(fp))
                data(ii,:) = textscan(fp, '%s %f %f %f %f %f %f %f %f %f %f');
                ii = ii + 1;
            end
            fclose(fp);
            
            time_stamp_all = data{2};
            xyz_all = [data{3},data{4},data{5}];
            quaternion_all = [data{9}, data{6}, data{7}, data{8}];
            
            time_stamp = time_stamp_all(robot_index:4:end);
            xyz = xyz_all(robot_index:4:end,:);
            
            % read quaternion and fix sign flip
            quaternion_raw = quaternion_all(robot_index:4:end,:);
            
            % keep the sign of the first quatertion
            quaternion(1, :) = quaternion_raw(1, :);
            
            % iterate to fix the sign
            sign_status = 0;
            for ii = 2 : length(quaternion_raw(:,1))
                % check adjacent quaternions are in different signs, if so,
                % flip th sign
                number_of_sign_changes = ...
                    sum(abs(sign(quaternion_raw(ii, :)) - sign(quaternion_raw(ii - 1, :))));
                quaternion_element_difference_max =  ...
                    max(quaternion_raw(ii, :) - quaternion_raw(ii - 1, :));
                
                if(number_of_sign_changes == 8 && sign_status == 0)
                    sign_status = 1;
                elseif(number_of_sign_changes == 8 && sign_status == 1)
                    sign_status = 0;
                elseif(number_of_sign_changes > 0 && abs(quaternion_element_difference_max) > 0.1)
                    sign_status = ~sign_status;
                end
                
                if(sign_status)
                    quaternion(ii, :) = - quaternion_raw(ii, :);
                else
                    quaternion(ii, :) = quaternion_raw(ii, :);
                end
            end
            figure;
            plot(time_stamp, xyz);
            legend('x','y','z');
            title('Position command on tip(m)');
            figure;
            plot(time_stamp, quaternion);
            legend('qw','qx','qy','qz');
            title('Quaternion command on tip');
            
            % default to no interpolation
            number_of_interpolations = 0;
            
            % interpolate data if delta_t is not zero
            if delta_t ~= 0
                % find all the index of non zero time differences
                time_diff = diff(time_stamp);
                idx = (time_diff ~= 0);
                number_of_interpolations = floor(mean(time_diff( idx )) / delta_t);
            end
            
            % expand the data with interpolations
            if( number_of_interpolations )
                % create array to store interpolated data
                data_items_int = length(time_stamp) * (number_of_interpolations + 1);
                time_stamp_int = zeros(data_items_int, 1);
                quaternion_int = zeros(data_items_int, 4);
                xyz_int = zeros(data_items_int, 3);
                
                % iterate to get interpolation points
                for ii = 1 : length(time_stamp) - 1
                    idx = (ii - 1) * (number_of_interpolations + 1) + 1 : ...
                        ii * (number_of_interpolations + 1);
                    time_stamp_int(idx) = obj.interpolate_linear_data(time_stamp(ii), ...
                        time_stamp(ii + 1), number_of_interpolations);
                    xyz_int(idx, :) = obj.interpolate_linear_data(xyz(ii, :), xyz(ii + 1, :),...
                        number_of_interpolations);
                    quaternion_int(idx, :) = obj.interpolate_quaternion(quaternion(ii, :), ...
                        quaternion(ii + 1, :), number_of_interpolations);
                end
            else
                time_stamp_int = time_stamp;
                quaternion_int = quaternion;
                xyz_int = xyz;
            end
            
            % calculate angular velocity and linear velocity
            for ii = 1: length(time_stamp_int) - 1
                dt = time_stamp_int(ii + 1) - time_stamp_int(ii);
                if(dt == 0.0)
                    angular_velocity(ii, :) = [0.0, 0.0, 0.0]; %#ok<*AGROW>
                    linear_velocity(ii,:) = [0.0, 0.0, 0.0];
                else
                    rotation_current = obj.quaternion_to_rotation_wxyz(quaternion_int(ii, :));
                    rotation_next = obj.quaternion_to_rotation_wxyz(quaternion_int(ii + 1, :));
                    linear_velocity(ii , :) = ( xyz_int(ii+1, :) - xyz_int(ii, :) ) / dt;
                    angular_velocity(ii, :) = ...
                        obj.calculate_angular_velocity_from_rotation_diff(...
                        rotation_current, rotation_next, dt);
                end
            end
            figure;
            plot(linear_velocity,'.');
            legend('vx','vy','vz');
            title('Task space linear velocities(m/s)');
            figure;
            plot( angular_velocity,'.');
            legend('wx','wy','wz');
            title('Task space angular velocities(rad/s)');
        end
        
    end
end