function calibrated_dh = KinematicCalibrationStage1(joint_angle_file, ...
    tip_pos_file, tip_rot_file, varargin)
  % function calibrated_dh = KinematicCalibrationStage1(joint_angle_file, ...
  % tip_pos_file, tip_rot_file, varargin)
  % This function calibrated the dh parameters for J1 ~ J6, and the translation of the marker origin
  % to RCM position.
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

% create a KinematicCalibrationClass instance
kc_obj = KinematicCalibrationClass();

% load data
joint_angles_measured = load(joint_angle_file);
tool_angles = zeros(length(joint_angles_measured), 4);
joint_angles_measured = [joint_angles_measured, tool_angles];
tip_xyz_measured = load(tip_pos_file);
tip_rotation_measured = load(tip_rot_file);
tip_xyz_measured = tip_xyz_measured / KinematicCalibrationClass.M2MM;

% add dh parameters
TIP_JOINT_INDEX = 18;
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

% the following calculation is used to get the initial estimate from the
% tip to the joint 6(spherical roll) transformation
% 
% pinv(kc_obj.robot_object_.frames_(:, :, 7)) * 
% kc_obj.robot_object_.frames_(:, :, 18) * 
% [1.0, 0.0,0.0,0.0; 0.0,1.0,0.0,0.0; 0.0, 0.0, 1.0, 0.18388609; 0.0,0.0,0.0,1.0]
%

tip_2_rcm = [1.0, 0.0, 0.0, 0.0; 
             0.0, 1.0, 0.0, 0.0; 
             0.0, 0.0, 1.0, 0.184; 
             0.0, 0.0, 0.0, 1.0];

tip_2_rcm_xyz = tip_2_rcm( 1 : 3, 4);
tip_2_rcm_pry = kc_obj.compute_euler_angles_from_rotation_matrix(tip_2_rcm(1:3, 1:3)) ;
tip_2_rcm_params = [tip_2_rcm_xyz', tip_2_rcm_pry];
tip_2_rcm_params_calibrated = tip_2_rcm_params; % initialization
tip_2_rcm_param_mask = [1, 1, 1, 0, 0, 0]; % first 3 are xyz, the second 3 are pry

% intialized the calibrated dh paramreters to nominal
dh_parameters_calibrated = dh_parameters;

% convert the dh parameters into cell array of struct
for i = 1:length(dh_parameters)
    dh_parameters_struct.dh_params{i} =  cell2struct(dh_parameters{i}, ...
        dh_fieldnames, 2);
    
    dh_calibrate_mask_index_struct.mask{i} =  ...
        cell2struct(dh_parameters_calibrate_index_mask{i}, dh_fieldnames, 2);
    
    % scale the dh properly
    dh_parameters_struct.dh_params{i}.link_length = ...
        dh_parameters_struct.dh_params{i}.link_length * ...
        kc_xyz_scale;
    
    dh_parameters_struct.dh_params{i}.link_offset = ...
        dh_parameters_struct.dh_params{i}.link_offset * ...
        kc_xyz_scale;

end



% calculate forward kinematics for nominal and disturbed dh parameters
% pre-allocate some transformation matrices
total_point_count = size(joint_angles_measured, 1);

% [urdf_frames_original, urdf_frames_dh, urdf_frames_converted] = ...
%             CalculateFKConvertDH2URDF(kc_obj.robot_object_);
rng(10);
random_index = randperm(size(joint_angles_measured, 1));
calibraton_point_count = floor(total_point_count * 0.9);
verification_point_count = total_point_count - calibraton_point_count;

calibration_point_index = random_index(1 : calibraton_point_count);
verification_point_index = random_index(calibraton_point_count + 1 : end);

joint_angles_measured_cal = joint_angles_measured(calibration_point_index, :);
tip_xyz_measured_cal = tip_xyz_measured(calibration_point_index, :);
tip_rotation_measured_cal = tip_rotation_measured(calibration_point_index, :);

joint_angles_measured_ver = joint_angles_measured(verification_point_index, :);
tip_xyz_measured_ver = tip_xyz_measured(verification_point_index, :);
tip_rotation_measured_ver = tip_rotation_measured(verification_point_index, :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%%
centroid_measured_cal = mean(tip_xyz_measured_cal);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% collect the nominal parameters for calibration
dh_calibrate_count = 1;
dh_to_calibrate = zeros(length(dh_parameters) * 4 + ...
    EXTRA_PARAMETERS_TO_CALIBRATE_MAX, 1);
for i = 1 : length(dh_parameters)
    dh_joint = dh_parameters_struct.dh_params{i};   
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
for i = 1 : 6
    if(tip_2_rcm_param_mask(i) == 1)
        dh_to_calibrate(dh_calibrate_count) = tip_2_rcm_params(i);
        dh_calibrate_count = dh_calibrate_count + 1;
    end
end

% shrink the dh to be calibrate vector to the right length
dh_to_calibrate(dh_calibrate_count : end) = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% caliculate the tip position/orientation with nominal DH parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tip_xyz_nominal_cal = zeros(calibraton_point_count, 3);
tip_x_axis_nominal_cal = zeros(calibraton_point_count, 3);
tip_y_axis_nominal_cal = zeros(calibraton_point_count, 3);
for i = 1 : calibraton_point_count
    % calculate the forward kinematics
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured_cal(i, :)');
    rcm_pose_nominal= kc_obj.robot_object_.frames_(:, :, TIP_JOINT_INDEX);
    tip_pose_nominal = rcm_pose_nominal * tip_2_rcm;
    
    tip_xyz_nominal_cal(i, :) = tip_pose_nominal(1 : 3, 4)';
    tip_x_axis_nominal_cal(i, :) = tip_pose_nominal(1 : 3, 1)';
    tip_y_axis_nominal_cal(i, :) = tip_pose_nominal(1 : 3, 2)';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tip_xyz_nominal_ver = zeros(verification_point_count, 3);
tip_x_axis_nominal_ver = zeros(verification_point_count, 3);
tip_y_axis_nominal_ver = zeros(verification_point_count, 3);
for i = 1 : verification_point_count
    % calculate the forward kinematics
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured_ver(i, :)');
    rcm_pose_ver= kc_obj.robot_object_.frames_(:, :, TIP_JOINT_INDEX);
    tip_pose_ver = rcm_pose_ver * tip_2_rcm;
    
    tip_xyz_nominal_ver(i, :) = tip_pose_ver(1 : 3, 4)';
    tip_x_axis_nominal_ver(i, :) = tip_pose_ver(1 : 3, 1)';
    tip_y_axis_nominal_ver(i, :) = tip_pose_ver(1 : 3, 2)';
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define options for the optimization
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'InitDamping', 1.0e-2, 'MaxIter', 10, 'MaxFunEvals', 800, ...
    'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
    'TolX', 1.0e-9, 'TolFun', 1.0e-7, ...
    'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
    'Display','iter-detailed');

% define the cost function using function callback
opt_type = 'xyz';
cost_function = @(x)kc_obj.CalculateCostFunction(x, kc_obj, ...
    dh_calibrate_mask_index_struct, ...
    joint_angles_measured_cal, tip_xyz_measured_cal, ...
    tip_rotation_measured_cal, TIP_JOINT_INDEX, ...
    tip_2_rcm_params, ...
    tip_2_rcm_param_mask, ...
    centroid_measured_cal, ...
    opt_type);

% call the optimization function

[dh_calibrated, resnorm, residual, exitflag, output, lambda, jacobian] = ...
    lsqnonlin(cost_function, dh_to_calibrate, [], [], options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : length(dh_parameters)
    dh_parameters_calibrated{i} = ...
        {kc_obj.robot_object_.dh_parameters_modified_{i}.link_twist,...
        kc_obj.robot_object_.dh_parameters_modified_{i}.link_length,...
        kc_obj.robot_object_.dh_parameters_modified_{i}.link_offset,...
        kc_obj.robot_object_.dh_parameters_modified_{i}.joint_offset};
end

calibrated_dh = cell2mat(reshape([dh_parameters_calibrated{:}], 4, 14)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate the tip transformation matrix with the calibrated parameters
tip_xyz_calibrated = zeros(calibraton_point_count, 3);
tip_x_axis_calibrated = zeros(calibraton_point_count, 3);
tip_y_axis_calibrated = zeros(calibraton_point_count, 3);

num_tip_params = 0;
for i = 6 : -1 : 1
    if(tip_2_rcm_param_mask(i) == 1)
        tip_2_rcm_params_calibrated(i) = dh_calibrated(end - num_tip_params);
        num_tip_params = num_tip_params + 1;
    else
        tip_2_rcm_params_calibrated(i) = tip_2_rcm_params(i);
    end
end

% write dh to a file
save dhp_stage1.mat calibrated_dh tip_2_rcm_params_calibrated -double

calibrated_dh(:, 1) = calibrated_dh(:, 1) * 180 / pi;
calibrated_dh(:, 4) = calibrated_dh(:, 4) * 180 / pi;
disp(calibrated_dh(1 : 6 , :));

%
disp(tip_2_rcm_params_calibrated);

tip_2_rcm_rot_calibrated = ...
    RotationAxisAngle([0; 0; 1], tip_2_rcm_params_calibrated(6))*...
    RotationAxisAngle([0; 1; 0], tip_2_rcm_params_calibrated(4))*...
    RotationAxisAngle([1; 0; 0], tip_2_rcm_params_calibrated(5));
tip_2_rcm_pos_calibrated = tip_2_rcm_params_calibrated(1 : 3);

tip_2_rcm_calibrated = eye(4);
tip_2_rcm_calibrated(1:3, 1:3) = tip_2_rcm_rot_calibrated;
tip_2_rcm_calibrated(1:3, 4) = tip_2_rcm_pos_calibrated';

for i = 1 : calibraton_point_count
    % calculate the forward kinematics
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured_cal(i, :)');
    rcm_pose_calibrated= kc_obj.robot_object_.frames_(:, :, TIP_JOINT_INDEX);
    tip_pose_calibrated = rcm_pose_calibrated * tip_2_rcm_calibrated;
    
    tip_xyz_calibrated(i, :) = tip_pose_calibrated(1 : 3, 4)';
    tip_x_axis_calibrated(i, :) = tip_pose_calibrated(1 : 3, 1)';
    tip_y_axis_calibrated(i, :) = tip_pose_calibrated(1 : 3, 2)';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tip_xyz_ver_calibrated = zeros(verification_point_count, 3);
tip_x_axis_ver_calibrated = zeros(verification_point_count, 3);
tip_y_axis_ver_calibrated = zeros(verification_point_count, 3);
for i = 1 : verification_point_count
    % calculate the forward kinematics
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured_ver(i, :)');
    rcm_pose_ver= kc_obj.robot_object_.frames_(:, :, TIP_JOINT_INDEX);
    tip_pose_ver_calibrated = rcm_pose_ver * tip_2_rcm_calibrated;
    
    tip_xyz_ver_calibrated(i, :) = tip_pose_ver_calibrated(1 : 3, 4)';
    tip_x_axis_ver_calibrated(i, :) = tip_pose_ver_calibrated(1 : 3, 1)';
    tip_y_axis_ver_calibrated(i, :) = tip_pose_ver_calibrated(1 : 3, 2)';
end

% calculate rms error
[rms_error_calibrated_cal, individual_error_calibrated_cal] = ...
    kc_obj.CalculateRMSEToCentroid(tip_xyz_calibrated, tip_xyz_measured_cal);
[rms_error_calibrated_ver, individual_error_calibrated_ver] = ...
    kc_obj.CalculateRMSEToCentroid(tip_xyz_ver_calibrated, tip_xyz_measured_ver);

[rms_error_nominal_cal, individual_error_nominal_cal] = ...
    kc_obj.CalculateRMSEToCentroid(tip_xyz_nominal_cal, tip_xyz_measured_cal);
[rms_error_nominal_ver,individual_error_nominal_ver] = ...
    kc_obj.CalculateRMSEToCentroid(tip_xyz_nominal_ver, tip_xyz_measured_ver);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
plot(individual_error_calibrated_cal,'*');
hold on;
plot(individual_error_calibrated_ver,'o');
plot([individual_error_nominal_cal; individual_error_nominal_ver],'x');
hold off
title('Relative error - distance to centroid');
ylabel('relative error (m)')
legend( ' calibrate data', 'verification data', 'uncalibrated data');
grid minor;

figure;
tip_xyz_diff = tip_xyz_calibrated - tip_xyz_nominal_cal;
tip_xyz_diff_norm = sqrt(sum(tip_xyz_diff.^2, 2));
plot(tip_xyz_diff_norm, '*');
title('Distance between calibrated and uncalibrated points');
ylabel('distance(m)');
grid minor;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
camera_2_robot = kc_obj.FindTransformationMatrixBetweenReferenceFrames(...
    tip_xyz_measured_cal, tip_xyz_calibrated);
tip_xyz_measured_arm_calibrated = (camera_2_robot * ...
    [tip_xyz_measured_cal, ones(length(tip_xyz_measured_cal), 1)]')';
tip_xyz_measured_arm_calibrated = tip_xyz_measured_arm_calibrated(:, 1 : 3);

camera_2_robot = kc_obj.FindTransformationMatrixBetweenReferenceFrames(...
    tip_xyz_measured_cal, tip_xyz_nominal_cal);
tip_xyz_measured_arm_nominal = (camera_2_robot * ...
    [tip_xyz_measured_cal, ones(length(tip_xyz_measured_cal), 1)]')';
tip_xyz_measured_arm_nominal = tip_xyz_measured_arm_nominal(:, 1 : 3);

figure;
plot3(tip_xyz_nominal_cal(:,1), tip_xyz_nominal_cal(:,2), ...
    tip_xyz_nominal_cal(:,3), 'b*');
hold on;
plot3(tip_xyz_calibrated(:,1), tip_xyz_calibrated(:,2), ...
    tip_xyz_calibrated(:,3), 'r*');
plot3(tip_xyz_measured_arm_nominal(:,1), tip_xyz_measured_arm_nominal(:,2), ...
    tip_xyz_measured_arm_nominal(:,3), 'bd');
plot3(tip_xyz_measured_arm_calibrated(:,1), tip_xyz_measured_arm_calibrated(:,2), ...
    tip_xyz_measured_arm_calibrated(:,3), 'rd');
hold off
title('Points plot using camera-robot registration');
legend('uncalibrated-robot','calibrated-robot','uncalibrated-camera',...
    'calibrated-camera','Location','Best');
grid minor;

figure;
tip_xyz_diff = tip_xyz_measured_arm_calibrated - tip_xyz_calibrated;
tip_xyz_diff_norm = sqrt(sum(tip_xyz_diff.^2, 2));
plot(tip_xyz_diff_norm, '*');
hold on;
tip_xyz_diff = tip_xyz_measured_arm_nominal - tip_xyz_nominal_cal;
tip_xyz_diff_norm = sqrt(sum(tip_xyz_diff.^2, 2));
plot(tip_xyz_diff_norm, 'd');
hold off;
title('Relative error between points using camera-robot registration');
ylabel('Error(m)');
legend('calibrated','uncalibrated');
grid minor;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end of the main functions
end