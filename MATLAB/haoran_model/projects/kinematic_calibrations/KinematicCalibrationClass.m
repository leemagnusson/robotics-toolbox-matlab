classdef KinematicCalibrationClass < handle
    % classdef KinematicCalibrationClass < handle
    % This class provides functions and implementations for kinematic calibration
    % the constructor of the class take V1.5 as default, but could be expanded to support V2.0 arm
    % in the future.
    % kc_obj = KinematicCalibrationClass()
    % kc_obj = KinematicCalibrationClass('1.0')
    % additional function to support conert DH to XYZ_RPY is implemented in this class
    
    % constant properties
    properties(Constant)
        M2MM = 1000;
        M2CM = 100;
        M2DM = 10;
    end
    % private properties
    properties(Access = private)
        dh_parameters_modified_;
    end
    % piublic properties
    properties(Access = public)
        robot_object_;
        vertex_arm_origin_;
    end
    
    %public constructors
    methods(Access = public)
        function obj = KinematicCalibrationClass(varargin)
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
                    name_urdf_file = 'V1.5_Arm_URDF_new.URDF';
                    name_stl_folder = strcat(folder_name,'meshes_new/');
                    name_full_urdf_path= strcat(folder_name, 'robots/',name_urdf_file);
                    urdf_input = URDFParser(name_full_urdf_path);
                    urdf_link_input = urdf_input.links;
                    urdf_joint_input = urdf_input.joints;
                    urdf_joint_sequence = urdf_input.jseq; %#ok<NASGU>
                    
                    % create a robot object
                    obj.robot_object_ = RobotClass(urdf_link_input,urdf_joint_input);
                    % call the calculate forward kinematic to create all
                    % the frames
                    obj.robot_object_.CalculateFK(zeros(length(urdf_link_input), 1));
                    
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
            for i = 1:length(obj.robot_object_.name_)
                if exist(strcat(name_stl_folder,...
                        obj.robot_object_.stl_name_{i}), 'file') == 2
                    vertex_arm_origin(:, i) =  stl2matlab(strcat(...
                        name_stl_folder, obj.robot_object_.stl_name_{i})); %#ok<*AGROW>
                else
                    vertex_arm_origin(:, i) = {[];[];[]};
                end
            end
            obj.vertex_arm_origin_ = vertex_arm_origin;
        end
    end
    % other public methods
    methods(Access = public)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
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
        
        % function to set dh parameters
        function SetDHParametersModified(obj, dhp_in)
            obj.robot_object_.dh_parameters_modified_ = dhp_in;
        end
    end
    
    % define some static methods
    methods(Static)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % supporting functions
        %
        function draw_robot_zero_pose(kc_obj)
            %kc_obj.robot_object_.transformation_base_ = eye(4);
            kc_obj.robot_object_.CalculateFKDHModified(zeros(11,1));
            
            % draw robot
            kc_obj.robot_object_.DrawRobot(kc_obj.vertex_arm_origin_, ...
                [2:18], 0.6 , 0.4);
            
            % setup plot properties
            axis([-0.5, 0.5, -0.5, 0.8, -0.5, 0.5]);
            light('Position',[1 3 -3]);
            light('Position',[1 -3 3]);
            view(40, 10);
            ax = gca;
            ax.Color = [0.98, 1, 1]; % lightcyan
            %ax.Box = 'on';
            %ax.BoxStyle = 'full';
            ax.LineWidth = 0.1;
            drawnow;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % this function is for cost function callback
        function f_out = CalculateCostFunction(dh_to_calibrate, obj, ...
                dh_mask_index_struct, joint_angles, tip_xyz_measured, ...
                tip_rot_measured, ...
                tip_frame_index, ...
                tip_2_rcm_params_step, ...
                tip_2_rcm_param_mask_step, ...
                centroid_measured,...
                opt_type)
            % get a copy of the dh paramters
            dh_params_full = obj.robot_object_.dh_parameters_modified_;
            
            % update the dh parameters with the intermediate values
            % formulate the disturbed parameters for calibration
            dh_count = 1;
            for ii = 1 : length(dh_params_full)
                % The order of assignment has to be fixed, so do not change.
                if(dh_mask_index_struct.mask{ii}.link_twist)
                    dh_params_full{ii}.link_twist  = dh_to_calibrate(dh_count);
                    dh_count = dh_count + 1;
                end
                
                if(dh_mask_index_struct.mask{ii}.link_length)
                    dh_params_full{ii}.link_length = dh_to_calibrate(dh_count);
                    dh_count = dh_count + 1;
                end
                
                if(dh_mask_index_struct.mask{ii}.link_offset)
                    dh_params_full{ii}.link_offset = dh_to_calibrate(dh_count);
                    dh_count = dh_count + 1;
                end
                
                if(dh_mask_index_struct.mask{ii}.joint_offset)
                    dh_params_full{ii}.joint_offset = dh_to_calibrate(dh_count);
                    dh_count = dh_count + 1;
                end
            end
            
            % get the tip to rcm parmamters if they are part of the
            % calibration parameters
            tip_param_cnt = 0;
            for ii = 6 : -1 : 1
                if(tip_2_rcm_param_mask_step(ii) == 1)
                    tip_2_rcm_params_step(ii) = ...
                        dh_to_calibrate(end - tip_param_cnt);
                    tip_param_cnt = tip_param_cnt + 1;
                end
            end
            
            % get the tip to spherical roll transformation matrix
            tip_2_rcm_rot = ...
                RotationAxisAngle([0; 0; 1], tip_2_rcm_params_step(6))*...
                RotationAxisAngle([0; 1; 0], tip_2_rcm_params_step(4))*...
                RotationAxisAngle([1; 0; 0], tip_2_rcm_params_step(5));
            tip_2_rcm_pos = tip_2_rcm_params_step(1 : 3);
            
            tip_2_rcm_step = eye(4);
            tip_2_rcm_step(1:3, 1:3) = tip_2_rcm_rot;
            tip_2_rcm_step(1:3, 4) = tip_2_rcm_pos';
            
            % update the dh paramters as a whole
            obj.robot_object_.dh_parameters_modified_ = dh_params_full;
            
            % iterate to calculate the mse
            point_counts = size(joint_angles, 1);
            tip_x_axis_step = zeros(point_counts, 3);
            tip_y_axis_step = zeros(point_counts, 3);
            tip_z_axis_step = zeros(point_counts, 3);
            tip_xyz_step = zeros(point_counts, 3);
            f_out = zeros(1, point_counts);
            
            for ii = 1 : point_counts
                % calculate the forward kinematics
                obj.robot_object_.CalculateFKDHModified(joint_angles(ii, :)');
                spherical_roll_pose_step= obj.robot_object_.frames_(:, :, tip_frame_index);
                tip_pose_step = spherical_roll_pose_step * tip_2_rcm_step;
                
                tip_xyz_step(ii, :) = tip_pose_step(1 : 3, 4)';
                tip_x_axis_step(ii, :) = tip_pose_step(1 : 3, 1)';
                tip_y_axis_step(ii, :) = tip_pose_step(1 : 3, 2)';
                tip_z_axis_step(ii, :) = tip_pose_step(1 : 3, 3)';
            end
            
            % calculate the sphere center
            centroid_step = mean(tip_xyz_step);
            
            for ii = 1 : point_counts
                switch opt_type
                    case 'xyz'
                        error_vector = norm(tip_xyz_step(ii, :) - ...
                            centroid_step) - ...
                            norm(tip_xyz_measured(ii, : ) - centroid_measured);
                    otherwise
                        error('Unsupported optimization type received');
                end
                f_out(ii) = f_out(ii) + norm(error_vector);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [urdf_frames_original, urdf_frames_dh, urdf_frames_converted] = ...
                CalculateFKConvertDH2URDF(robot_obj)
            % calculate forward kinematics from original urdf
            robot_obj.CalculateFK(zeros(11, 1));
            urdf_frames_original = robot_obj.frames_;
            
            % calculate forward kinematics from dh
            robot_obj.CalculateFKDHModified(zeros(11, 1));
            urdf_frames_dh = robot_obj.frames_;
            
            % convert dh to urdf and update robot
            % thi function does not change the urdf file, only update the joint
            % xyz and rpy of the instance
            robot_obj.CalculateXYZAndFixedFrameEulerAnglesFromDH();
            
            % recalculate forward kinematics from updated urdf
            robot_obj.CalculateFK(zeros(11, 1));
            urdf_frames_converted = robot_obj.frames_;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [center, radius] = EstimateSphereCenterRadius3D(xyz_in)
            point_count = length(xyz_in);
            
            A = zeros(point_count, 4);
            b = zeros(point_count, 1);
            for ii = 1 : length(xyz_in)
                A(ii, :) = [-2.0 * xyz_in(ii, :), 1.0];
                b(ii) = sum(xyz_in(ii, :).^2);
            end
            
            x = pinv(A) * b;
            center = x(1:3);
            radius = sqrt(sum(center.^2) - x(4));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function inverse_T = InverseTransformationMatrix(T)
            inverse_T = [ T(1:3, 1:3)', -T(1:3, 1:3)' * T(1:3, 4); ...
                0, 0, 0, 1 ];
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
        function [rms_error, individual_errors] = CalculateRMSEToCentroid(points_1, points_2)
            % check number of points are same
            num_points = length(points_1);
            if length(points_2) ~= num_points
                error('number of points mismatch');
            end
            
            % calculate the centroids and relative vectors
            centroid_1 = mean(points_1);
            centroid_2 = mean(points_2);
            points_1_relative = points_1 - repmat(centroid_1, num_points, 1);
            points_2_relative = points_2 - repmat(centroid_2, num_points, 1);
            
            % calculate the distance error
            rms_error = 0.0;
            individual_errors = zeros(num_points , 1);
            
            for ii = 1 : num_points
                individual_errors(ii) = ...
                    norm(points_1_relative(ii, :)) - norm(points_2_relative(ii, :));
                rms_error = rms_error + abs(individual_errors(ii));
            end
            rms_error = rms_error / num_points;
        end
        function transformation_matrix = ...
                FindTransformationMatrixBetweenReferenceFrames(point_set_a, point_set_b)
            % This function finds the transformation between two reference
            % frames, where same point set are represented.
            % Inputs : point_set_a and point_set_b are same points represented in two
            % different reference frames, both are n by 3 matrix
            % Output: tranformation_matrix from frame a to frame b
            
            % check dimensions
            num_points = length(point_set_a);
            
            if(length(point_set_b) ~= num_points)
                error('invlaid length of point set a and b, needs to be equal');
            end
            
            % find centroid of each point set
            centroid_a = mean(point_set_a);
            centroid_b = mean(point_set_b);
            
            % find the covariance matrix
            cov_matrix = (point_set_a' - repmat(centroid_a', 1, num_points)) * ...
                (point_set_b - repmat(centroid_b, num_points, 1));
            
            [uu, ~, vv] = svd(cov_matrix);
            
            % find rotation
            rr = vv * uu';
            
            % find translation
            tt = (- rr * centroid_a' + centroid_b');
            
            % compose the transformation matrix
            transformation_matrix = [rr, tt; 0, 0, 0, 1];
        end
        
        function [center,radius,residuals] = spherefit(x,y,z)
            % Fit a sphere to data using the least squares approach.
            %
            % Fits the equation of a sphere in Cartesian coordinates to a set of xyz
            % data points by solving the overdetermined system of normal equations, i.e.
            % x^2 + y^2 + z^2 + a*x + b*y + c*z + d = 0
            % The least squares sphere has radius R = sqrt((a^2+b^2+c^2)/4-d) and
            % center coordinates (x,y,z) = (-a/2,-b/2,-c/2).
            %
            % Input arguments:
            % x,y,z:
            %    Cartesian coordinates of noisy data points
            %
            % Output arguments:
            % center:
            %    coordinates of the least-squares fit sphere center
            % radius:
            %    least-squares fit sphere radius
            % residuals:
            %    residuals in the radial direction
            %
            % Examples:
            % [center,radius,residuals] = shperefit(X)
            % [center,radius,residuals] = spherefit(x,y,z);
            
            % Copyright 2010 Levente Hunyadi
            
            narginchk(1,3);
            n = size(x,1);
            switch nargin  % n x 3 matrix
                case 1
                    validateattributes(x, {'numeric'}, {'2d','real','size',[n,3]});
                    z = x(:,3);
                    y = x(:,2);
                    x = x(:,1);
                otherwise  % three x,y,z vectors
                    validateattributes(x, {'numeric'}, {'real','vector'});
                    validateattributes(y, {'numeric'}, {'real','vector'});
                    validateattributes(z, {'numeric'}, {'real','vector'});
                    x = x(:);  % force into columns
                    y = y(:);
                    z = z(:);
                    validateattributes(x, {'numeric'}, {'size',[n,1]});
                    validateattributes(y, {'numeric'}, {'size',[n,1]});
                    validateattributes(z, {'numeric'}, {'size',[n,1]});
            end
            
            % need four or more data points
            if n < 4
                error('spherefit:InsufficientData', ...
                    'At least four points are required to fit a unique sphere.');
            end
            
            % solve linear system of normal equations
            A = [x, y, z, ones(size(x))];
            b = -(x.^2 + y.^2 + z.^2);
            a = A \ b;
            
            % return center coordinates and sphere radius
            center = -a(1:3)./2;
            radius = realsqrt(sum(center.^2)-a(4));
            
            if nargout > 2
                % calculate residuals
                residuals = radius - sqrt(sum(bsxfun(@minus,[x y z],center.').^2,2));
            elseif nargout > 1
                % skip
            else
                % plot sphere
                hold all;
                sphere_gd(6,radius,center);
                hold off;
            end            
        end        
    end
end