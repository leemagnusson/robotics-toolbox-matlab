function KinematicCalibrationSimulationSpherical(varargin)
  % function KinematicCalibrationSimulationSpherical(varargin)
  % This function use nominal and a disturbed dh to simulate the kinematic calibration algorithm
  % for J7 of the arm.
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
SPHERICAL_BASE_INDEX = 6;
EXTRA_PARAMETERS_TO_CALIBRATE_MAX = 18; % upto two fixed transformations

kc_xyz_scale = 1.0;
dh_parameters_struct.arm_version = 'V1.5';
dh_fieldnames = RobotClass.DH_FIELDNAMES;
dh_parameters_struct.dh_params = [];
dh_parameters = RobotClass.DH_PARAMS;
dh_parameters_calibrated = RobotClass.DH_PARAMS;
dh_parameters_calibrate_index_mask = {...
    {0, 0, 0, 0};...% shoulder pitch
    {0, 0, 0, 0};...% shoulder roll 
    {0, 0, 0, 0};...% elbow pitch
    {0, 0, 0, 0};...% elbow roll
    {0, 0, 0, 0};...% spherical base
    {0, 0, 0, 0};...% spherical roll
    {1, 1, 1, 1};...% spherical pitch a
    {1, 1, 1, 1};...% spherical pitch b
    {1, 1, 1, 1};...% spherical pitch c
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

twist_bound = 1 /180 * pi; % 1 degrees
link_length_bound = 0.0005; % 1 mm
link_offset_bound = 0.0005; % 1 mm
joint_offset_bound = 1 / 180 * pi; % 2 degree

spherical_dh_distrubance = {...
    { twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % shoulder pitch
    { twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % shoulder roll 
    { twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % elbow pitch
    { twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % elbow roll
    { twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % spherical base
    {-twist_bound, -link_length_bound, -link_offset_bound, -joint_offset_bound}, ... % spherical roll
    { twist_bound, -link_length_bound, -link_offset_bound, -joint_offset_bound}, ... % spherical pitch a
    {-twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % spherical pitch b
    {-twist_bound, -link_length_bound, -link_offset_bound,  joint_offset_bound}, ... % spherical pitch c
    { twist_bound,  link_length_bound,  link_offset_bound,  joint_offset_bound}, ... % tool translate
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool roll
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool priximal wrist
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool distal wrist(gripper a)
    { 0.000,  -0.000,  0.000,  0.000}, ... % tool distal wrist(gripper b)    
    };

for i = 1 : length(spherical_dh_distrubance)
%     if(i < 7 || i > 9)
        dh_parameters_disturbed{i} = num2cell(cell2mat(dh_parameters_disturbed{i}) ...
            + cell2mat(spherical_dh_distrubance{i}) .* ...
            cell2mat(dh_parameters_calibrate_index_mask{i}));
%     else
%         dh_parameters_disturbed{i} = num2cell(cell2mat(dh_parameters_disturbed{i}) ...
%             + cell2mat(spherical_dh_distrubance{i}));
%     end
end

% intialized the calibrated dh paramreters to nominal
dh_parameters_calibrated = dh_parameters;

% joint angles 
angle_increment = 0.2; % rad
q6 = - pi/2 : angle_increment : pi/2; % +-4.7124 rad
q7 = - pi/4 : angle_increment : pi/4;
% q7b = -1.4069 : angle_increment : 1.0154;
% q7c = -1.0154 : angle_increment : 1.4069;
%q8 = -0.081825 : 0.03: 0.17; % m
q8 = 0.0;
% scale the prismatic joint
q8 = q8 * kc_xyz_scale;

q6_index = randperm(length(q6));
q6_shuffled = q6(q6_index);

q7_index = randperm(length(q7));
q7_shuffled = q7(q7_index);
% q7b_shuffled = q7b(q7_index);
% q7c_shuffled = q7c(q7_index);

q8_index = randperm(length(q8));
q8_shuffled = q8(q8_index);

%q9 = [-pi/6 ,0.0, pi/6];
q9 = 0.0;

% gennerate joint angle combinations
joint_angles = zeros(length(q6) * length(q7) * length(q8), 11);
for i = 1: length(q6)
    for j = 1: length(q7)
        for k = 1: length(q8)
            for p = 1 : length(q9)
                point_index = p + (k - 1) * length(q9) + ...
                    (j - 1) * length(q8) * length(q9) + ...
                    (i - 1) * length(q7) * length(q8) * length(q9);
                joint_angles(point_index, :) = [[0.0, 0.0, 0.0, 0.0, 0.0]'; [q6_shuffled(i), ...
                    q7_shuffled(j), q8_shuffled(k), q9(p)]';zeros(2,1)];
            end
        end        
    end
end

% tip_tracker_2_tracker_base = eye(4);
spherical_base_tracker_2_spherical_base_nominal = eye(4);
spherical_base_tracker_2_spherical_base_nominal(1 : 3, 4) = [0.05, 0.1, 0.1]';

spherical_base_tracker_2_spherical_base_disturbance=...
    [0, 0 ,0, 0.005;
     0, 0, 0, 0.005;
     0, 0, 0, 0.005;
     0, 0, 0, 0];
spherical_base_tracker_2_spherical_base_disturbed = ...
    spherical_base_tracker_2_spherical_base_nominal;

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

% [urdf_frames_original, urdf_frames_dh, urdf_frames_converted] = ...
%             CalculateFKConvertDH2URDF(kc_obj.robot_object_);
random_index = randperm(size(joint_angles, 1));
calibraton_point_count = floor(total_point_count * 0.75);
joint_angles_measured = joint_angles(random_index(1 : calibraton_point_count), :);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set disturbed dh parameters, these dh parameters are treated as tru value
% of the robot
kc_obj.robot_object_.dh_parameters_modified_ = ...
    dh_parameters_disturbed_struct.dh_params;

tip_pose_measured =  ones(4, 4, calibraton_point_count);
tip_xyz_measured = zeros(calibraton_point_count, 3);
tip_x_axis_measured = zeros(calibraton_point_count, 3);
tip_y_axis_measured = zeros(calibraton_point_count, 3);
tip_z_axis_measured = zeros(calibraton_point_count, 3);
tip_tracker_2_base_tracker = ones(4, 4, calibraton_point_count);

% calculate the uncalibrated data as measurement
for i = 1 : calibraton_point_count
    % calculate forward kinematic
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured(i, :)');
    
    % get tool tip transformation to the base tracker, which is then used
    % as the base(ground truth) for calibration
    tip_pose_measured(:, : ,i) = kc_obj.robot_object_.frames_(: , : , TIP_XYZ_INDEX);
    spherical_base_pose_measured = ...
        kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX);    
    
    tip_tracker_2_spherical_base =  ...
        kc_obj.robot_object_.frames_(: , : , TIP_XYZ_INDEX) * ...
        InverseTransformationMatrix(spherical_base_pose_measured);
    
    tip_tracker_2_base_tracker(:, :, i) = tip_tracker_2_spherical_base *...
        InverseTransformationMatrix(spherical_base_tracker_2_spherical_base_nominal);
    
    tip_xyz_measured(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 4 , i);
    tip_x_axis_measured(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 1 , i);
    tip_y_axis_measured(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 2 , i);
    tip_z_axis_measured(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 3 , i);    
end

tip_axis_measured.x_ = tip_x_axis_measured;
tip_axis_measured.y_ = tip_y_axis_measured;
tip_axis_measured.z_ = tip_z_axis_measured;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%%
[center_measured, radius_measured, ~] = spherefit(tip_xyz_measured(:, 1), ...
    tip_xyz_measured(: , 2), tip_xyz_measured(: , 3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get nominal dh forward kinematics
kc_obj.robot_object_.dh_parameters_modified_ = dh_parameters_struct.dh_params;
% get the nominal values
tip_pose_nominal =  ones(4, 4, calibraton_point_count);
tip_xyz_nominal = zeros(calibraton_point_count, 3);
tip_x_axis_nominal = zeros(calibraton_point_count, 3);
tip_y_axis_nominal = zeros(calibraton_point_count, 3);
tip_z_axis_nominal = zeros(calibraton_point_count, 3);
spherical_base_pose_nominal = ones(4, 4, calibraton_point_count);

for i = 1 : calibraton_point_count
    % calculate forward kinematic for comparison
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured(i, :)');
    
    % get tool tip transformation to the base tracker, which is then used
    % as the base(ground truth) for calibration
    tip_pose_nominal(:, : ,i) = kc_obj.robot_object_.frames_(: , : , TIP_XYZ_INDEX);
    spherical_base_pose_nominal = ...
        kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX);    
    
    tip_tracker_2_spherical_base =  ...
        kc_obj.robot_object_.frames_(: , : , TIP_XYZ_INDEX) * ...
        InverseTransformationMatrix(kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX));
    
    tip_tracker_2_base_tracker(:, :, i) = tip_tracker_2_spherical_base *...
        InverseTransformationMatrix(spherical_base_tracker_2_spherical_base_nominal);
    
    tip_xyz_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 4 , i);
    tip_x_axis_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 1 , i);
    tip_y_axis_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 2 , i);
    tip_z_axis_nominal(i, :) =  tip_tracker_2_base_tracker( 1 : 3, 3 , i);    
end

tip_axis_nominal.x_ = tip_x_axis_nominal;
tip_axis_nominal.y_ = tip_y_axis_nominal;
tip_axis_nominal.z_ = tip_z_axis_nominal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5%%%%
[center_nominal, radius_nominal, ~] = spherefit(tip_xyz_nominal(:, 1), ...
    tip_xyz_nominal(: , 2), tip_xyz_nominal(: , 3));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% collect the nominal parameters for calibration
dh_calibrate_count = 1;
dh_to_calibrate = zeros(length(dh_parameters) * 4 + EXTRA_PARAMETERS_TO_CALIBRATE_MAX, 1);
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
for i = 1 : 4
    for j = 1:4
        if(spherical_base_tracker_2_spherical_base_mask(i, j) == 1)
            dh_to_calibrate(dh_calibrate_count) = ...
                spherical_base_tracker_2_spherical_base_nominal(i, j);
            dh_calibrate_count = dh_calibrate_count + 1;
        end
    end
end

% shrink the dh to be calibrate vector to the right length
dh_to_calibrate(dh_calibrate_count : end) = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define options for the optimization
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'InitDamping', 1.0e2, 'MaxIter', 15, 'MaxFunEvals', 800, ...
    'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
    'TolX', 1.0e-9, 'TolFun', 1.0e-7, ...
    'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
    'Display','iter-detailed');

% define the cost function using function callback
opt_type = 'sphere';
cost_function = @(x)CalculateCostFunction(x, kc_obj, dh_calibrate_mask_index_struct, ...
    joint_angles_measured, tip_xyz_measured, tip_axis_measured,TIP_XYZ_INDEX, ...
    spherical_base_tracker_2_spherical_base_disturbed, ...
    spherical_base_pose_nominal,...
    spherical_base_tracker_2_spherical_base_mask, ...
    radius_measured, center_measured, ...
    opt_type);

% call the optimization function

[dh_calibrated, resnorm,residual,exitflag,output,lambda,jacobian] = ...
    lsqnonlin(cost_function, dh_to_calibrate, [], [], options);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define options for the optimization
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'InitDamping', 1.0e2, 'MaxIter', 30, 'MaxFunEvals', 800, ...
    'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
    'TolX', 1.0e-9, 'TolFun', 1.0e-5, ...
    'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
    'Display','iter-detailed');

% define the cost function using function callback
opt_type = 'x_axis';
cost_function = @(x)CalculateCostFunction(x, kc_obj, dh_calibrate_mask_index_struct, ...
    joint_angles_measured, tip_xyz_measured, tip_axis_measured, TIP_XYZ_INDEX, ...
    spherical_base_tracker_2_spherical_base_disturbed,...
    spherical_base_pose_nominal,...
    spherical_base_tracker_2_spherical_base_mask, ...
    radius_measured, center_measured, opt_type);

% call the optimization function

[dh_calibrated, resnorm,residual,exitflag,output,lambda,jacobian] = ...
    lsqnonlin(cost_function, dh_calibrated, [], [], options); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define options for the optimization
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',...
    'InitDamping', 5.0e2, 'MaxIter', 30, 'MaxFunEvals', 800, ...
    'FinDiffType', 'forward', 'FinDiffRelStep', 1.0e-12, ...
    'TolX', 1.0e-9, 'TolFun', 1.0e-7, ...
    'DiffMinChange', 0.0, 'DiffMaxChange', inf,...
    'Display','iter-detailed');

% define the cost function using function callback
opt_type = 'combined';
cost_function = @(x)CalculateCostFunction(x, kc_obj, dh_calibrate_mask_index_struct, ...
    joint_angles_measured, tip_xyz_measured, tip_axis_measured, TIP_XYZ_INDEX, ...
    spherical_base_tracker_2_spherical_base_disturbed, ...
    spherical_base_pose_nominal,...
    spherical_base_tracker_2_spherical_base_mask, ...
    radius_measured, center_measured, opt_type);

% call the optimization function

[dh_calibrated, resnorm,residual,exitflag,output,lambda,jacobian] = ...
    lsqnonlin(cost_function, dh_calibrated, [], [], options); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate the tip transformation matrix with the calibrated parameters
tip_xyz_calibrated = zeros(calibraton_point_count, 3);
tip_x_axis_calibrated = zeros(calibraton_point_count, 3);
tip_y_axis_calibrated = zeros(calibraton_point_count, 3);

% % update the transormation matrix
% for i = 1 : 4
%     for j = 1 : 4
%         if(spherical_base_tracker_2_spherical_base_mask(i, j) == 1)
%             spherical_base_tracker_2_spherical_base_disturbed(i, j) = ...
%                 dh_calibrated(dh_param_count);
%             dh_param_count = dh_param_count + 1;
%         end
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : length(dh_parameters)
    dh_parameters_calibrated{i} = {kc_obj.robot_object_.dh_parameters_modified_{i}.link_twist,...
        kc_obj.robot_object_.dh_parameters_modified_{i}.link_length,...
        kc_obj.robot_object_.dh_parameters_modified_{i}.link_offset,...
        kc_obj.robot_object_.dh_parameters_modified_{i}.joint_offset};
end

calibrated_dh = reshape([dh_parameters_calibrated{:}], 4, 14)';
true_dh = reshape([dh_parameters_disturbed{:}], 4, 14)';
disp(calibrated_dh(7:9,:));
disp(true_dh(7:9,:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : calibraton_point_count
    % calculate the forward kinematics
    kc_obj.robot_object_.CalculateFKDHModified(joint_angles_measured(i, :)');
    tip_pose_calibrted= kc_obj.robot_object_.frames_(:, :, TIP_XYZ_INDEX);
    
    tip_2_spherical_base_calibrated = tip_pose_calibrted * ...
        InverseTransformationMatrix(kc_obj.robot_object_.frames_(: , : , SPHERICAL_BASE_INDEX));
    
    tip_tracker_2_base_tracker_calibrated = tip_2_spherical_base_calibrated *...
        InverseTransformationMatrix(spherical_base_tracker_2_spherical_base_disturbed);
    
    tip_xyz_calibrated(i, :) = tip_tracker_2_base_tracker_calibrated(1 : 3, 4)';
    tip_x_axis_calibrated(i, :) = tip_tracker_2_base_tracker_calibrated(1 : 3, 1)';
    tip_y_axis_calibrated(i, :) = tip_tracker_2_base_tracker_calibrated(1 : 3, 2)';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[center_calibrated, radius_calibrated, ~] = spherefit(tip_xyz_calibrated(:, 1), ...
    tip_xyz_calibrated(: , 2), tip_xyz_calibrated(: , 3));

[ux, uy, uz ]= sphere(200);
surf(ux * radius_calibrated + center_calibrated(1), uy * radius_calibrated + center_calibrated(2), ...
    uz * radius_calibrated + center_calibrated(3), 'FaceColor', [0.7 0.7 0.7], 'edgeColor',[0, 0, 0],...
    'facealpha', 0.85, 'edgealpha', 0.0);
axis equal;
hold on;
plot3(tip_xyz_measured(:,1), tip_xyz_measured(:,2), ...
    tip_xyz_measured(:,3),'g+','MarkerSize', 5.0);

plot3(tip_xyz_nominal(:,1), tip_xyz_nominal(:,2),tip_xyz_nominal(:,3), '*');
hold on;
plot3(tip_xyz_calibrated(:,1), tip_xyz_calibrated(:,2),tip_xyz_calibrated(:,3), 'ro');
hold off;

% calculate the distance from the measured/calibrated points to sphere
distanceToSphereSurfaceUncalibrated = zeros(size(tip_xyz_nominal, 1), 1);
for i = 1 : size(tip_xyz_nominal, 1)
    distanceToSphereSurfaceUncalibrated(i) = norm(tip_xyz_nominal(i, :) - center_measured') - radius_measured;
end

figure;
marker_size = 5;
plot(distanceToSphereSurfaceUncalibrated * RobotClass.M2MM,'b*', 'MarkerSize', marker_size);

distanceToSphereSurfaceCalibrated = zeros(size(tip_xyz_calibrated, 1), 1);
for i = 1 : size(tip_xyz_calibrated, 1)
    distanceToSphereSurfaceCalibrated(i) = norm(tip_xyz_calibrated(i, :) - center_calibrated') - radius_calibrated;
end
hold on;
plot(distanceToSphereSurfaceCalibrated * RobotClass.M2MM,'g*', 'MarkerSize', marker_size);
title('Distances from the points to estimated rcm sphere surface');
ylabel('Signed distance(mm)');
legend('Uncalibrated', 'Calibrated');
hold off;

center_calibrated - center_measured
radius_calibrated - radius_measured

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this function is for cost function callback
    function f_out = CalculateCostFunction(dh_to_calibrate, obj, ...
            dh_mask_index_struct, joint_angles, tip_xyz_measured, ...
            tip_axis_measured, ...
            tip_frame_index, ...
            spherical_base_tracker_2_spherical_base_disturbed,...
            spherical_base_t, ...
            spherical_base_tracker_2_spherical_base_mask, ...
            radius, center,...
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
            
            spherical_base_t_inv =InverseTransformationMatrix(spherical_base_t);
            tip_2_spherical_base = tip_pose_intermediate * spherical_base_t_inv;
    
            spherical_base_tracker_2_spherical_base_disturbed_inv = ...
                InverseTransformationMatrix(spherical_base_tracker_2_spherical_base_disturbed);

            tip_tracker_2_base_tracker_calib = tip_2_spherical_base *...
                spherical_base_tracker_2_spherical_base_disturbed_inv;

            tip_xyz_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 4)';
            tip_x_axis_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 1)';
            tip_y_axis_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 2)';
            tip_z_axis_intermediate(ii, :) = tip_tracker_2_base_tracker_calib(1 : 3, 3)';
        end
        
        % calculate the sphere center
        [center_intermediate, radius_intermediate, ~] = spherefit(tip_xyz_intermediate(:, 1), ...
            tip_xyz_intermediate(: , 2), tip_xyz_intermediate(: , 3));
        
        for ii = 1 : point_counts
            switch opt_type
                case 'xyz'
                    error_vector = (tip_xyz_intermediate(ii, :) - ...
                        center_intermediate' -  ...
                        (tip_xyz_measured(ii, : ) - center'));
                    
                case 'xyz_0'
                    error_vector = (tip_xyz_intermediate(ii, :) - ...
                        tip_xyz_intermediate(1, :) -  ...
                        (tip_xyz_measured(ii, : ) - tip_xyz_measured(1, : )));

                case 'sphere'
                    error_vector = norm(tip_xyz_intermediate(ii, :) - ...
                        center') - radius + ...
                        norm(center_intermediate - center) + radius_intermediate -...
                        radius;
                    
                case 'x_axis'
                    error_vector = tip_x_axis_intermediate(ii, :) - ...
                        tip_x_axis_intermediate(1, :) - ...
                        (tip_axis_measured.x_(ii, :) - tip_axis_measured.x_(1, :));
                    
                case 'y_axis'
                    error_vector = tip_y_axis_intermediate(ii, :) -  ...
                        tip_y_axis_intermediate(1, :) - ...
                        (tip_axis_measured.y_(ii, :) - tip_axis_measured.y_(1, :));
                    
                case 'z_axis'
                    error_vector = tip_z_axis_intermediate(ii, :) -  ...
                        tip_z_axis_intermediate(1, :) - ...
                        (tip_axis_measured.x_(ii, :) - tip_axis_measured.y_(1, :));
                    
                case 'xy_axis'
                    error_vector(1:3) = tip_x_axis_intermediate(ii, :) - ...
                        tip_x_axis_intermediate(1, :) - ...
                        (tip_axis_measured.x_(ii, :) - tip_axis_measured.x_(1, :));
                    error_vector(4:6) = tip_y_axis_intermediate(ii, :) -  ...
                        tip_y_axis_intermediate(1, :) - ...
                        (tip_axis_measured.y_(ii, :) - tip_axis_measured.y_(1, :));
                    
                case 'combined'
                    error_vector(1:3) = (tip_xyz_intermediate(ii, :) - ...
                        tip_xyz_intermediate(1, :)) -  ...
                        (tip_xyz_measured(ii, : ) - tip_xyz_measured(1, : ));
                    error_vector(4:6) = tip_x_axis_intermediate(ii, :) - ...
                        tip_x_axis_intermediate(1, :) - ...
                        (tip_axis_measured.x_(ii, :) - tip_axis_measured.x_(1, :));
                    error_vector(7:9) = tip_y_axis_intermediate(ii, :) - ...
                        tip_y_axis_intermediate(1, :) - ...
                        (tip_axis_measured.y_(ii, :) - tip_axis_measured.y_(1, :));
                    error_vector(10:12) = tip_z_axis_intermediate(ii, :) - ...
                        tip_z_axis_intermediate(1, :) - ...
                        (tip_axis_measured.z_(ii, :) - tip_axis_measured.z_(1, :));

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

end