function KinematicCalibrationSimulationCartesian(varargin)
  % function KinematicCalibrationSimulationCartesian(varargin)
  % This function use nominal and a disturbed dh to simulate the kinematic calibration algorithm
  % for J1 ~ J6 of the arm
  % the simulation is for V1.5 arm
  %
% add pathes for libraries and data
addpath('../../lib');
addpath('../../lib/common');
addpath('../../lib/math');
addpath('../../lib/plot');
addpath('../../lib/robot');
addpath('../../data');
addpath('../../export');
addpath('../../init');
addpath('../../projects');
addpath('../../workspace');
addpath('../../procedure_ui');


fclose('all');
close('all');

% load data


% add dh parameters
TIP_XYZ_INDEX = 13;
SPHERICAL_BASE_INDEX = 1;
EXTRA_PARAMETERS_TO_CALIBRATE_MAX = 18; % upto two fixed transformations

kc_xyz_scale = 1.0;
dh_parameters_struct.arm_version = 'V1.5';
dh_fieldnames = RobotClass.DH_FIELDNAMES;
dh_parameters_struct.dh_params = [];
dh_parameters = RobotClass.DH_PARAMS;
dh_parameters_calibrate_index_mask = {...
    {0, 0, 0, 1};...% shoulder pitch
    {1, 1, 1, 1};...% shoulder roll 
    {1, 1, 1, 1};...% elbow pitch
    {1, 1, 1, 1};...% elbow roll
    {1, 1, 1, 1};...% spherical base
    {1, 1, 1, 1};...% spherical roll
    {0, 0, 0, 0};...% spherical pitch a
    {0, 0, 0, 0};...% spherical pitch b
    {0, 0, 0, 0};...% spherical pitch c
    {0, 0, 0, 0};...% tool translate
    {0, 0, 0, 0};...% tool roll
    {0, 0, 0, 0};...% tool priximal wrist
    {0, 0, 0, 0};...% tool distal wrist(gripper a)
    {0, 0, 0, 0};...% tool distal wrist(gripper b)  
    };

spherical_base_tracker_2_spherical_base_mask = [0, 0 ,0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];

dh_parameters_disturbed_struct.arm_version = 'V1.5';
dh_parameters_disturbed_struct.dh_params = [];

dh_parameters_disturbed = RobotClass.DH_PARAMS;

spherical_dh_distrubance = {...
    { 0.005,  -0.005,  0.005,  0.005}, ... % shoulder pitch
    { 0.005,  -0.005,  0.005,  0.005}, ... % shoulder roll 
    { 0.005,  -0.005,  0.005,  0.005}, ... % elbow pitch
    { 0.005,  -0.005,  0.005,  0.005}, ... % elbow roll
    { 0.005,  -0.005,  0.005,  0.005}, ... % spherical base
    { 0.005,  -0.005,  0.005, -0.005}, ... % spherical roll
    {-0.005,  -0.005,  0.005,  0.005}, ... % spherical pitch a
    {-0.005,   0.005,  0.005,  0.005}, ... % spherical pitch b
    { 0.005,   0.005, -0.005, -0.005}, ... % spherical pitch c
    {-0.005,  -0.005,  0.005, -0.005}, ... % tool translate
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool roll
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool priximal wrist
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool distal wrist(gripper a)
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool distal wrist(gripper b)    
    };

twist_bound = 2 /180 * pi; % 2degrees
link_length_bound = 0.002; % 2mm
link_offset_bound = 0.002; % 2mm
joint_offset_bound = 4 / 180 * pi; % 5 degree

for i = 1 : length(spherical_dh_distrubance)
    dh_parameters_disturbed{i} = num2cell(cell2mat(dh_parameters_disturbed{i}) ...
        + cell2mat(spherical_dh_distrubance{i}) .* ...
        cell2mat(dh_parameters_calibrate_index_mask{i}));
end

% joint angles 
angle_increment = pi/12; % rad
angle_range = [pi/8, pi/8, pi/8, pi/8,pi/6, pi/6];
q1 = -angle_range(1)/2 : angle_increment/2 : angle_range(1)/2; % +-4.7124 rad
q2 = -angle_range(2) : angle_increment : angle_range(2);
q3 = -angle_range(3) : angle_increment : angle_range(3); % +-4.7124 rad
q4 = -angle_range(4) : angle_increment : angle_range(4);
q5 = -angle_range(5) : angle_increment : angle_range(5); % +-4.7124 rad
q6 = -angle_range(6) : angle_increment : angle_range(6);

q1_index = randperm(length(q1));
q1_shuffled = q6(q1_index);
q2_index = randperm(length(q2));
q2_shuffled = q6(q2_index);
q3_index = randperm(length(q3));
q3_shuffled = q6(q3_index);
q4_index = randperm(length(q4));
q4_shuffled = q6(q4_index);
q5_index = randperm(length(q5));
q5_shuffled = q6(q5_index);
q6_index = randperm(length(q6));
q6_shuffled = q6(q6_index);

% gennerate joint angle combinations
joint_angles = zeros(length(q6) * length(q5) * length(q4) * length(q3) * length(q2) * length(q1), 11);
for i = 1: length(q1)
    for j = 1: length(q2)
        for k = 1: length(q3)
            for m = 1 : length(q4)
                for n = 1: length(q5)
                    for p = 1: length(q6)
                        point_index = p + ...
                            (n - 1) * length(q6) + ...
                            (m - 1) * length(q6) * length(q5) + ...
                            (k - 1) * length(q6) * length(q5) * length(q4) + ...
                            (j - 1) * length(q6) * length(q5) * length(q4) * length(q3) + ...
                            (i - 1) * length(q6) * length(q5) * length(q4) * length(q3) * length(q2);
                        joint_angles(point_index, :) = [[q1_shuffled(i), ...
                            q2_shuffled(j), q3_shuffled(k), q4_shuffled(m), q5_shuffled(n), ...
                            q6_shuffled(p)]';zeros(5,1)];
                    end
                end
            end
        end        
    end
end

% tip_tracker_2_tracker_base = eye(4);
spherical_base_tracker_2_spherical_base_nominal = eye(4);
% spherical_base_tracker_2_spherical_base_nominal(1 : 3, 4) = [0.05, 0.1, 0.1]';
spherical_base_tracker_2_spherical_base_disturbed = ...
    spherical_base_tracker_2_spherical_base_nominal;
spherical_base_tracker_2_spherical_base_noise= [0, 0 ,0, 0.005;
    0, 0, 0, 0.005;
    0, 0, 0, 0.005;
    0, 0, 0, 0];



% convert the dh parameters into cell array of struct
for i = 1:length(dh_parameters)
    dh_parameters_struct.dh_params{i} =  cell2struct(dh_parameters{i}, ...
        dh_fieldnames, 2);
    
    dh_parameters_disturbed_struct.dh_params{i} =  ...
        cell2struct(dh_parameters_disturbed{i}, dh_fieldnames, 2);
    
    dh_calibrate_mask_index_struct.mask{i} =  ...
        cell2struct(dh_parameters_calibrate_index_mask{i}, dh_fieldnames, 2);
    
    % scale the dh properly
    dh_parameters_struct.dh_params{i}.link_length = ...
        dh_parameters_struct.dh_params{i}.link_length * ...
        kc_xyz_scale;
    dh_parameters_struct.dh_params{i}.link_offset = ...
        dh_parameters_struct.dh_params{i}.link_offset * ...
        kc_xyz_scale;
    
    dh_parameters_disturbed_struct.dh_params{i}.link_length = ...
        dh_parameters_disturbed_struct.dh_params{i}.link_length * ...
        kc_xyz_scale;
    
    dh_parameters_disturbed_struct.dh_params{i}.link_offset = ...
        dh_parameters_disturbed_struct.dh_params{i}.link_offset * ...
        kc_xyz_scale;

end

% create a KinematicCalibrationClass instance
switch nargin
    case 0
        kc_obj = KinematicCalibrationClass();
    case 1
        kc_obj = KinematicCalibrationClass(varargin{1});
    otherwise
        error('Invalid input, TestKCClass function expect zero or one input');
end

% calculate forward kinematics for nominal and disturbed dh parameters
% pre-allocate some transformation matrices
total_point_count = size(joint_angles, 1);


% get nominal dh forward kinematics as the ground truth
kc_obj.robot_object_.dh_parameters_modified_ = dh_parameters_struct.dh_params;

% [urdf_frames_original, urdf_frames_dh, urdf_frames_converted] = ...
%             CalculateFKConvertDH2URDF(kc_obj.robot_object_);
random_index = randperm(size(joint_angles, 1));
calibraton_point_count = floor(total_point_count * 0.1);
joint_angles_calibrate = joint_angles(random_index(1 : calibraton_point_count), :);

tip_pose_nominal =  ones(4, 4, calibraton_point_count);
% tip_pose_disturbed = ones(4, 4, calibraton_point_count);
tip_xyz_nominal = zeros(calibraton_point_count, 3);
tip_x_axis_nominal = zeros(calibraton_point_count, 3);
tip_y_axis_nominal = zeros(calibraton_point_count, 3);
tip_z_axis_nominal = zeros(calibraton_point_count, 3);
% tip_xyz_disturbed = zeros(calibraton_point_count, 3);
tip_tracker_2_base_tracker = ones(4, 4, calibraton_point_count);
spherical_base_pose_nominal = ones(4, 4, calibraton_point_count);
% spherical_base_pose_disturbed = ones(4, 4, calibraton_point_count);

for i = 1 : calibraton_point_count
    % calculate forward kinematic
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_calibrate(i, :)');
    
    % get tool tip transformation to the base tracker, which is then used
    % as the base(ground truth) for calibration
    tip_pose_nominal(:, : ,i) = kc_obj.robot_object_.frames_(: , : , TIP_XYZ_INDEX);
    spherical_base_pose_nominal = ...
        kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX);    
    
    tip_tracker_2_spherical_base =  ...
        kc_obj.robot_object_.frames_(: , : , TIP_XYZ_INDEX) * ...
        pinv(kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX));
    
    tip_tracker_2_base_tracker(:, :, i) = tip_tracker_2_spherical_base *...
        pinv(spherical_base_tracker_2_spherical_base_nominal);
    
    tip_xyz_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 4 , i);
    tip_x_axis_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 1 , i);
    tip_y_axis_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 2 , i);
    tip_z_axis_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 3 , i);
    
end

tip_axis_nominal.x_ = tip_x_axis_nominal;
tip_axis_nominal.y_ = tip_y_axis_nominal;
tip_axis_nominal.z_ = tip_z_axis_nominal;

% set disturbed dh parameters
kc_obj.robot_object_.dh_parameters_modified_ = ...
    dh_parameters_disturbed_struct.dh_params;

% formulate the disturbed parameters for calibration
dh_calibrate_count = 1;
dh_to_calibrate = zeros(length(dh_parameters) * 4 + EXTRA_PARAMETERS_TO_CALIBRATE_MAX);
for i = 1 : length(dh_parameters)
    dh_joint = dh_parameters_disturbed_struct.dh_params{i};   
    % the order of assignment is fixed, so do not change
    if(dh_calibrate_mask_index_struct.mask{i}.link_twist)
        dh_to_calibrate(dh_calibrate_count) = dh_joint.link_twist;
        dh_calibrate_count = dh_calibrate_count + 1;
    end
    if(dh_calibrate_mask_index_struct.mask{i}.link_length)
        dh_to_calibrate(dh_calibrate_count) = dh_joint.link_length;
        dh_calibrate_count = dh_calibrate_count + 1;
    end
    
    if(dh_calibrate_mask_index_struct.mask{i}.link_offset)
        dh_to_calibrate(dh_calibrate_count) = dh_joint.link_offset;
        dh_calibrate_count = dh_calibrate_count + 1;
    end
    
    if(dh_calibrate_mask_index_struct.mask{i}.joint_offset)
        dh_to_calibrate(dh_calibrate_count) = dh_joint.joint_offset;
        dh_calibrate_count = dh_calibrate_count + 1;
    end
end

% fill in extra parameters to calibrate
for i = 1 : 4
    for j = 1:4
        if(spherical_base_tracker_2_spherical_base_mask(i, j) == 1)
            % update the disturbed base transformation
            spherical_base_tracker_2_spherical_base_disturbed(i, j ) = ...
            spherical_base_tracker_2_spherical_base_noise(i, j) + ...
                spherical_base_tracker_2_spherical_base_nominal(i, j);
            % fill in the parameter to be calibrated 
            dh_to_calibrate(dh_calibrate_count) = ...
                spherical_base_tracker_2_spherical_base_disturbed(i, j);
            dh_calibrate_count = dh_calibrate_count + 1;
        end
    end
end

% shrink the dh to be calibrate vector to the right length
dh_to_calibrate(dh_calibrate_count : end) = [];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define options for the optimization
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'InitDamping', 1.0e2, 'MaxIter', 30, 'MaxFunEvals', 800, ...
    'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
    'TolX', 1.0e-9, 'TolFun', 1.0e-7, ...
    'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
    'Display','iter-detailed');

% define the cost function using function callback
opt_type = 'xyz';
cost_function = @(x)CalculateCostFunction(x, kc_obj, dh_calibrate_mask_index_struct, ...
    joint_angles_calibrate, tip_xyz_nominal, tip_axis_nominal,TIP_XYZ_INDEX, ...
    spherical_base_tracker_2_spherical_base_disturbed, ...
    spherical_base_pose_nominal,...
    spherical_base_tracker_2_spherical_base_mask, opt_type);

% call the optimization function

[dh_calibrated, resnorm,residual,exitflag,output,lambda,jacobian] = ...
    lsqnonlin(cost_function, dh_to_calibrate, [], [], options);
disp(dh_calibrated);

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % define options for the optimization
% options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
%     'InitDamping', 1.0e2, 'MaxIter', 30, 'MaxFunEvals', 800, ...
%     'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
%     'TolX', 1.0e-9, 'TolFun', 1.0e-7, ...
%     'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
%     'Display','iter-detailed');
% 
% % define the cost function using function callback
% opt_type = 'y_axis';
% cost_function = @(x)CalculateCostFunction(x, kc_obj, dh_calibrate_mask_index_struct, ...
%     joint_angles_calibrate, tip_xyz_nominal, tip_axis_nominal, TIP_XYZ_INDEX, ...
%     spherical_base_tracker_2_spherical_base_disturbed,...
%     spherical_base_pose_nominal,...
%     spherical_base_tracker_2_spherical_base_mask, opt_type);
% 
% % call the optimization function
% 
% [dh_calibrated, resnorm,residual,exitflag,output,lambda,jacobian] = ...
%     lsqnonlin(cost_function, dh_calibrated, [], [], options); 
% disp(dh_calibrated);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % define options for the optimization
% options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
%     'InitDamping', 5.0e2, 'MaxIter', 30, 'MaxFunEvals', 800, ...
%     'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
%     'TolX', 1.0e-9, 'TolFun', 1.0e-7, ...
%     'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
%     'Display','iter-detailed');
% 
% % define the cost function using function callback
% opt_type = 'combined';
% cost_function = @(x)CalculateCostFunction(x, kc_obj, dh_calibrate_mask_index_struct, ...
%     joint_angles_calibrate, tip_xyz_nominal, tip_axis_nominal, TIP_XYZ_INDEX, ...
%     spherical_base_tracker_2_spherical_base_disturbed, ...
%     spherical_base_pose_nominal,...
%     spherical_base_tracker_2_spherical_base_mask, opt_type);
% 
% % call the optimization function
% 
% [dh_calibrated, resnorm,residual,exitflag,output,lambda,jacobian] = ...
%     lsqnonlin(cost_function, dh_calibrated, [], [], options); 

% disp(dh_calibrated);
nominal_dh = reshape([dh_parameters{:}], 4,14)';
disp(nominal_dh);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate the tip transformation matrix with the calibrated parameters
tip_xyz_calibrated = zeros(calibraton_point_count, 3);
tip_x_axis_calibrated = zeros(calibraton_point_count, 3);
tip_y_axis_calibrated = zeros(calibraton_point_count, 3);

dh_param_count = 1;
for i = 1 : length(dh_parameters)
    % The order of assignment has to be fixed, so do not change.
    if(dh_calibrate_mask_index_struct.mask{i}.link_twist)
        dh_param_count = dh_param_count + 1;
    end
    
    if(dh_calibrate_mask_index_struct.mask{i}.link_length)
        dh_param_count = dh_param_count + 1;
    end
    
    if(dh_calibrate_mask_index_struct.mask{i}.link_offset)
        dh_param_count = dh_param_count + 1;
    end
    
    if(dh_calibrate_mask_index_struct.mask{i}.joint_offset)
        dh_param_count = dh_param_count + 1;
    end
end

% update the transormation matrix
for i = 1 : 4
    for j = 1 : 4
        if(spherical_base_tracker_2_spherical_base_mask(i, j) == 1)
            spherical_base_tracker_2_spherical_base_disturbed(i, j) = ...
                dh_calibrated(dh_param_count);
            dh_param_count = dh_param_count + 1;
        end
    end
end

for i = 1 : calibraton_point_count
    % calculate the forward kinematics
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_calibrate(i, :)');
    tip_pose_calibrted= kc_obj.robot_object_.frames_(:, :, TIP_XYZ_INDEX);
    
    tip_2_spherical_base_calibrated = tip_pose_calibrted * ...
        pinv(kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX));
    
    tip_tracker_2_base_tracker_calibrated = tip_2_spherical_base_calibrated *...
        pinv(spherical_base_tracker_2_spherical_base_disturbed);
    
    tip_xyz_calibrated(i, :) = tip_tracker_2_base_tracker_calibrated(1 : 3, 4)';
    tip_x_axis_calibrated(i, :) = tip_tracker_2_base_tracker_calibrated(1 : 3, 1)';
    tip_y_axis_calibrated(i, :) = tip_tracker_2_base_tracker_calibrated(1 : 3, 2)';
end

plot3(tip_xyz_nominal(:,1), tip_xyz_nominal(:,2),tip_xyz_nominal(:,3), '*');
hold on;
plot3(tip_xyz_calibrated(:,1), tip_xyz_calibrated(:,2),tip_xyz_calibrated(:,3), 'ro');
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end of the main functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% this function is for cost function callback
    function f_out = CalculateCostFunction(dh_to_calibrate, obj, ...
            dh_mask_index_struct, joint_angles, tip_xyz_nominal, ...
            tip_axis_nominal, ...
            tip_frame_index, ...
            spherical_base_tracker_2_spherical_base_disturbed,...
            spherical_base_t, ...
            spherical_base_tracker_2_spherical_base_mask, ...
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
        
        % update the transormation matrix
        for ii = 1 : 4
            for jj = 1 : 4
                if(spherical_base_tracker_2_spherical_base_mask(ii, jj) == 1)
                    spherical_base_tracker_2_spherical_base_disturbed(ii, jj) = ...
                        dh_to_calibrate(dh_count);
                    dh_count = dh_count + 1;
                end
            end
        end
        
        % update the dh paramters as a whole
        obj.robot_object_.dh_parameters_modified_ = dh_params_full;
        
        % iterate to calculate the mse
        point_counts = size(joint_angles, 1);
        tip_x_axis_intermediate = zeros(point_counts, 3);
        tip_y_axis_intermediate = zeros(point_counts, 3);
        tip_z_axis_intermediate = zeros(point_counts, 3);
        tip_xyz_intermediate = zeros(point_counts, 3);
        f_out = zeros(1, point_counts);
        for ii = 1 : point_counts
            % calculate the forward kinematics
            obj.robot_object_.CalculateFKDHModified(joint_angles(ii, :)');
            tip_pose_intermediate= obj.robot_object_.frames_(:, :, tip_frame_index);
            
            tip_2_spherical_base = tip_pose_intermediate * InverseTransformationMatrix(spherical_base_t);
    
            tip_tracker_2_base_tracker_calib = tip_2_spherical_base *...
                InverseTransformationMatrix(spherical_base_tracker_2_spherical_base_disturbed);

            tip_xyz_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 4)';
            tip_x_axis_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 1)';
            tip_y_axis_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 2)';
            tip_z_axis_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 3)';
            
            switch opt_type
                case 'xyz'
                    error_vector = (tip_xyz_intermediate(ii, :) - ...
                        tip_xyz_intermediate(1, :)) -  ...
                        (tip_xyz_nominal(ii, : ) - tip_xyz_nominal(1, : ) );
%                     error_vector = (tip_xyz_intermediate(ii, :)) -  ...
%                         (tip_xyz_nominal(ii, : ));
                    
                case 'x_axis'
                    error_vector = tip_x_axis_intermediate(ii, :) - ...
                        tip_x_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.x_(ii, :) - tip_axis_nominal.x_(1, :));
                    
                case 'y_axis'
                    error_vector = tip_y_axis_intermediate(ii, :) -  ...
                        tip_y_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.y_(ii, :) - tip_axis_nominal.y_(1, :));
                    
                case 'z_axis'
                    error_vector = tip_z_axis_intermediate(ii, :) -  ...
                        tip_z_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.x_(ii, :) - tip_axis_nominal.y_(1, :));
                    
                case 'xy_axis'
                    error_vector(1:3) = tip_x_axis_intermediate(ii, :) - ...
                        tip_x_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.x_(ii, :) - tip_axis_nominal.x_(1, :));
                    error_vector(4:6) = tip_y_axis_intermediate(ii, :) -  ...
                        tip_y_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.y_(ii, :) - tip_axis_nominal.y_(1, :));
                    
                case 'combined'
                    error_vector(1:3) = (tip_xyz_intermediate(ii, :) - ...
                        tip_xyz_intermediate(1, :)) -  ...
                        (tip_xyz_nominal(ii, : ) - tip_xyz_nominal(1, : ));
                    error_vector(4:6) = tip_x_axis_intermediate(ii, :) - ...
                        tip_x_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.x_(ii, :) - tip_axis_nominal.x_(1, :));
                    error_vector(7:9) = tip_y_axis_intermediate(ii, :) - ...
                        tip_y_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.y_(ii, :) - tip_axis_nominal.y_(1, :));
                    error_vector(10:12) = tip_z_axis_intermediate(ii, :) - ...
                        tip_z_axis_intermediate(1, :) - ...
                        (tip_axis_nominal.z_(ii, :) - tip_axis_nominal.z_(1, :));

                otherwise
                    error('Unsupported optimization type received');
            end
            f_out(ii) = f_out(ii) + norm(error_vector);
        end
    end

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
    function inverse_T = InverseTransformationMatrix(T)
        inverse_T = [ T(1:3, 1:3)', -T(1:3, 1:3)' * T(1:3, 4); ...
            0, 0, 0, 1 ];
    end
end