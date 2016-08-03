function InverseKinematicBenchmark(theta_init, tip_joint_index, total_time, ...
    solver_type, trajectory_type, tracking_mode, varargin)
%
close all

% task space velocity limits
linear_motion = 0.06; % m
angular_motion = pi/2; % rad

% wrist initial angle
wrist_angle_init = pi * 1/2; % rad

% create an inverse kinematic object
disp('Create robot object ...');
ik_obj = InverseKinematicsClass();
disp('Create Robot object done');

% get a copy of robot object
robot_object = ik_obj.robot_object_;

% set solver type
ik_obj.solver_type_ = solver_type;

% set default flags
% turn on debug
debug_on = true;

% default disable data logging
enable_data_log = false;

% check for optional flags, the flags are bit masked
% 0 - all flags off
% 1 - enable debug info(bit 1)
% 2 - enable data logging(bit 2)
% 4 ~ 256 - reserved for future (bit 3 ~ 8)
if ~isempty(varargin{1})
    debug_on = bitget(varargin{1}, 1);
    enable_data_log = bitget(varargin{1}, 2);
end

% update debug flag
ik_obj.debug_info_on_ = debug_on;

% load data for V1.5
arm_version = 'V1_5';
load('arm_version_1.5.mat');
coupling_matrix_c = load('coupling_matrix.mat');
coupling_matrix = coupling_matrix_c.coupling_matrix; %#ok<NASGU>
vertex_arm_origin_c = load('vertex_arm_origin_1.5.mat');
vertex_arm_origin = vertex_arm_origin_c.vertex_arm_origin;
[nr,nc] = size(vertex_arm_origin);
for i = 1: nr
    for j = 1: nc
        vertex_arm_origin{i,j}  = vertex_arm_origin{i,j} * ...
            ik_obj.length_unit_scale_;
    end
end

% assign the vertex data for plot
ik_obj.vertex_arm_origin_ = vertex_arm_origin;

% calculate the initial forward kinematics
robot_object.transformation_base_ = eye(4);
theta_init(8) = -0.07;
theta_init(10:11) = pi/2;
robot_object.CalculateFK(theta_init);

% scale the translation coordinates
for i = 1: length(robot_object.frames_)
    robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
        ik_obj.length_unit_scale_;
    robot_object.frames_in_parent_(1:3, 4, i) = ...
        robot_object.frames_in_parent_(1:3, 4, i) * ...
        ik_obj.length_unit_scale_;
end

% get the intitial pose, and set the target to initial pose for now
pose_init = robot_object.frames_(:, :, tip_joint_index);
pose_target = pose_init;

% generate target pose
supported_trajectory_types = {'rotate_x', 'rotate_y','rotate_z',...
'x_motion', 'y_motion','z_motion', 'rotate_x_and_x_motion', ...
'rotate_y_and_y_motion', 'rotate_z_and_z_motion', ...
'rotate_xyz_and_y_motion', 'move_to_rcm'};

switch trajectory_type
    case 'rotate_x'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10:11) = wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about x axis of tool tip frame 
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_x_angle = RotationAxisAngle([1, 0, 0], angular_motion);        
        pose_target(1:3, 1:3) =  pose_init(1:3, 1:3) * rotation_x_angle;
        
    case 'rotate_y'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about y axis of tool tip frame
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_y_angle = RotationAxisAngle([0, 1, 0], angular_motion);
        pose_target(1:3, 1:3) = pose_init(1:3, 1:3) * rotation_y_angle;
        
    case 'rotate_z'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about z axis of tool tip frame
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_z_angle = RotationAxisAngle([0, 0, 1], angular_motion);
        pose_target(1:3, 1:3) = pose_init(1:3, 1:3) * rotation_z_angle;
        
    case 'x_motion'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % move the tip along x axis
        pose_target(1, 4) =  pose_init(1, 4) + ...
            linear_motion * ik_obj.length_unit_scale_;
        
    case 'y_motion'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % move the tip along y axis
        pose_target(2, 4) =  pose_init(2, 4) + ...
            linear_motion * ik_obj.length_unit_scale_;
        
    case 'z_motion'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = -wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % move the tip along z axis
        pose_target(3, 4) =  pose_init(3, 4) + ...
            linear_motion * ik_obj.length_unit_scale_;
        
    case 'rotate_x_and_x_motion'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = -wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about x axis of tool tip frame and move along x
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_x_angle = RotationAxisAngle([1, 0, 0], angular_motion);        
        pose_target(1:3, 1:3) = pose_init(1:3, 1:3) * rotation_x_angle;
        pose_target(1, 4) =  pose_init(1, 4) + linear_motion * ik_obj.length_unit_scale_;
        
    case 'rotate_y_and_y_motion'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = -wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about y axis of tool tip frame and move along y
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_y_angle = RotationAxisAngle([0, 1, 0], angular_motion);
        pose_target(1:3, 1:3) = pose_init(1:3, 1:3) * rotation_y_angle;
        pose_target(2, 4) =  pose_init(2, 4) + linear_motion * ik_obj.length_unit_scale_;
        
    case 'rotate_z_and_z_motion'
                % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = -wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about z axis of tool tip frame and move along z
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_z_angle = RotationAxisAngle([0, 0, 1], angular_motion);
        pose_target(1:3, 1:3) = pose_init(1:3, 1:3) * rotation_z_angle;
        pose_target(3, 4) =  pose_init(3, 4) + linear_motion * ik_obj.length_unit_scale_;
        
    case 'rotate_xyz_and_y_motion'
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = -wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);
        pose_target = pose_init;
        % rotate 180 degree about y axis of tool tip frame and move along y
        % get target pose
        pose_target(1:3, 4) = pose_init(1:3, 4);
        rotation_xyz_angle = RotationAxisAngle([1, 1, 1], angular_motion);
        pose_target(1:3, 1:3) =  pose_init(1:3, 1:3) * rotation_xyz_angle;
        pose_target(2, 4) =  pose_init(2, 4) + linear_motion * ik_obj.length_unit_scale_;
        
    case 'move_to_rcm'
        % move the tip to rcm
        % calculate the initial forward kinematics
        robot_object.transformation_base_ = eye(4);
        theta_init(8) = -0.07;
        theta_init(10) = wrist_angle_init;
        theta_init(11) = wrist_angle_init;
        robot_object.CalculateFK(theta_init);
        
        % scale the translation coordinates
        for i = 1: length(robot_object.frames_)
            robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
            robot_object.frames_in_parent_(1:3, 4, i) = ...
                robot_object.frames_in_parent_(1:3, 4, i) * ...
                ik_obj.length_unit_scale_;
        end
        
        % get the intitial pose, and set the target to initial pose for now
        pose_init = robot_object.frames_(:, :, tip_joint_index);        
        pose_rcm =  robot_object.frames_(:, :, 18);
        pose_target = linear_motion * ik_obj.length_unit_scale_ * ...
            (pose_rcm - pose_init)/norm(pose_rcm - pose_init) + pose_init;
        
    otherwise
        msg = sprintf('Unsupported trajectory: ''%s''\nSupported types:\n', trajectory_type);
        for i = 1: length(supported_trajectory_types)
            msg = [msg, sprintf('''%s''\n', supported_trajectory_types{i})];
        end
        error(msg);
        
end
% get the linear velocities using linear interpolation
linear_velocity= (pose_target(1:3,4) - pose_init(1:3,4)) / total_time;

% find the angular velocity using the slerp of quaternion rotation
% first find the quoternion for initial and final rotation
% quaternion in the format of [x,y,z,w]
q_init = ik_obj.rotation_to_quaternion_wxyz(pose_init(1:3,1:3));
q_target = ik_obj.rotation_to_quaternion_wxyz(pose_target(1:3,1:3));
pose_current = pose_init;
joint_angles_prev = theta_init;


if(enable_data_log)
    % set up initial plots
    orientation_fig_hdl = figure;
    time_stamp_data= [0, 0];
    orientation_error_data = [zeros(1,3);zeros(1,3)];
    orientation_error_hdl = plot(time_stamp_data, orientation_error_data);
    legend('roll', 'pitch', 'yaw');
    title(sprintf('Orientation error-%s-%s',solver_type, tracking_mode));
    ylabel('orientation error(deg)');
    xlabel('time(s)');
    
    position_fig_hdl = figure;
    pose_data = [pose_current(1:3,4)';pose_current(1:3,4)'];
    position_hdl = plot(time_stamp_data, pose_data);
    title(sprintf('Position error-%s-%s',solver_type, tracking_mode));
    ylabel('position error(m)');
    xlabel('time(s)');
    legend('x', 'y', 'z');
    
    joint_angle_fig_hdl = figure;
    joint_angle_data = [theta_init(6:11)';theta_init(6:11)'];
    theta_hdl = plot(time_stamp_data, joint_angle_data);
    title(sprintf('Joint angles-%s-%s',solver_type, tracking_mode));
    legend('1', '2', '3', '4', '5', '6');
    ylabel('joint angles(rad or m)');
    xlabel('time(s)');
    
    joint_velocity_fig_hdl = figure;
    joint_velocity_data = [zeros(1,6);zeros(1,6)];
    joint_velocity_hdl = plot(time_stamp_data, joint_velocity_data);
    legend('v_1', 'v_2', 'v_3', 'v_4', 'v_5', 'v_6');
    title(sprintf('Joint velocities-%s-%s',solver_type, tracking_mode));
    ylabel('joint velocities(rad/s or m/s)');
    xlabel('time(s)');
    
    ik_time_fig_hdl = figure;
    ik_time_data = [zeros(1,1), zeros(1,1)];
    ik_time_hdl = plot(time_stamp_data, joint_velocity_data);
    title(sprintf('Computation time-%s-%s',solver_type, tracking_mode));
    ylabel('computation time(seconds)');
    xlabel('time(s)');
    
    twist_fig_hdl = figure;
    twist_data = [zeros(1,6); zeros(1,6)];
    twist_hdl = plot(time_stamp_data, twist_data);
    title(sprintf('Task space velocities-%s-%s',solver_type, tracking_mode));
    ylabel('velocities time(rad/s or m/s)');
    legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z');
    xlabel('time(s)');
end
%% draw robot
robot_fig_hdl = figure;
axis_range = [-0.04 0.08 -0.33 -0.21 0.21 0.34];
ik_obj.DrawRobot(axis_range);
hold on;
DrawCoordinateSystem([1.0 1.0 1.0] * 0.02 * ...
    ik_obj.length_unit_scale_, pose_target(1:3, 1:3), ...
    pose_target(1:3, 4),'rgb');
title(sprintf('Robot EE trajecotry-%s-%s',solver_type, tracking_mode));

% plot the command trajectory
line([pose_init(1, 4), pose_target(1, 4)], ...
    [pose_init(2, 4), pose_target(2, 4)], ...
    [pose_init(3, 4), pose_target(3, 4)], ...
    'LineStyle', '-', 'Linewidth', 2);

% setup plot properties
light('Position',[1 3 -3]);
light('Position',[1 -3 3]);
view(40, 10);
ax = gca;
ax.Color = [0.98, 1, 1]; % lightcyan
ax.Box = 'on';
ax.BoxStyle = 'full';
ax.LineWidth = 0.1;
drawnow;

% create an empty playback frame struct
steps_total = total_time * ik_obj.IK_SOLVER_RATE;
play_back_frames(1: steps_total) = struct('cdata',[], 'colormap',[]);

% display processing start
slerp_step_count = 1;
fprintf('Processing: %09d', slerp_step_count)

% start processing the trajectory
while(slerp_step_count < steps_total)
    % update processing count
    fprintf('\b\b\b\b\b\b\b\b\b\b %09d', slerp_step_count);
    % check if data log is enabled
    if(enable_data_log)
        % save the current frame for later use
        play_back_frames(slerp_step_count) = getframe;
    end
    
    % update joint angles
    theta_start = joint_angles_prev;
    
    % calculate the next quaternion using slerp
    q_current = ik_obj.rotation_to_quaternion_wxyz(pose_current(1:3,1:3));
    q_step = 1 / steps_total * slerp_step_count;
    [q_next, ~] = ik_obj.slerp_quaternion( q_init, q_target, q_step );
    
    % calculate the next position using linear interpolation
    xyz_next = linear_velocity / InverseKinematicsClass.IK_SOLVER_RATE * ...
        slerp_step_count;
    
    % compute commanded pose based on tracking mode
    switch tracking_mode
        case 'absolute'
            pose_command(1:3,4) = pose_init(1:3,4) + xyz_next;
            rotation_command =  ...
                ik_obj.quaternion_to_rotation_wxyz(q_next);
        case 'relative'
            pose_command(1:3,4) = pose_current(1:3,4) + ...
                linear_velocity / InverseKinematicsClass.IK_SOLVER_RATE;
            rotation_command =  ...
                ik_obj.quaternion_to_rotation_wxyz(q_next) * ...
                ik_obj.quaternion_to_rotation_wxyz(q_current)' * ...
                pose_current(1:3, 1:3);
        otherwise
            error('Invalid tracking mode input: %s ', tracking_mode);
    end
    
    % start timer
    tic
    
    
    % compute inverse kinematics
    [joint_angles, joint_velocities, twist, pose_current, ik_iteration_steps] = ...
        ik_obj.compute_inverse_kinematics(pose_command(1:3, 4), ...
        rotation_command, 1/InverseKinematicsClass.IK_SOLVER_RATE, ...
        joint_angles_prev, theta_start, tip_joint_index);  %#ok<ASGLU>
    process_time = toc;
        
    % save joint angle and velocity for next step
    joint_angles_prev = joint_angles;
    
    % check if data log is enabled
    if(enable_data_log)
        % store data into data array
        time_stamp_data = [time_stamp_data, ...
            slerp_step_count / InverseKinematicsClass.IK_SOLVER_RATE]; %#ok<*AGROW>
        angle_diff = ik_obj.calculate_angle_from_rotation(rotation_command, ...
            pose_current(1:3,1:3));
        orientation_error_data = [orientation_error_data; angle_diff' * 180/pi];
        pose_data = [ pose_data; ...
            (pose_command(1:3,4) - pose_current(1:3,4))' / ik_obj.length_unit_scale_ ];
        joint_angle_data = [joint_angle_data; ...
            joint_angles(end - InverseKinematicsClass.SPHERICAL_ARM_DOF + 1: end)'];
        joint_velocity_data = [joint_velocity_data; joint_velocities'];
        ik_time_data = [ik_time_data, process_time];
        twist_data = [twist_data; twist'];
    end
    % update slerp count
    slerp_step_count = slerp_step_count + 1;
    
    % update robot plot
    ik_obj.DrawRobot(axis_range);
    plot3(pose_current(1,4), pose_current(2,4), pose_current(3,4), '.', 'MarkerSize', 5);
    drawnow;
end

% check for is data logging is enabled
if(enable_data_log)
    
    fprintf('\nWriting vedio file...');
    
    % create a vedio object
    video_writer_obj = VideoWriter(fullfile(pwd, ...
        sprintf('results/%s_%s_%s_%s.mp4', solver_type, ...
        trajectory_type, tracking_mode,arm_version)));
    
    % write frames to the video writer
    open(video_writer_obj);
    for i = 1:length(play_back_frames)
        if(~isempty(play_back_frames(i).cdata))
            writeVideo(video_writer_obj, play_back_frames(i));
        end
    end
    close(video_writer_obj);
    
    
    % plot the final result and save as figure
    plot_data_range = 3:length(time_stamp_data);
    
    for i = 1: 3
        set(orientation_error_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', orientation_error_data(plot_data_range,i));
    end
    savefig(orientation_fig_hdl, fullfile(pwd, ...
        sprintf('results/orientation_error_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type,tracking_mode, arm_version)), 'compact');
    
    for i = 1:3
        set(position_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', pose_data(plot_data_range,i));
    end
    savefig(position_fig_hdl, fullfile(pwd, ...
        sprintf('results/position_error_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type, tracking_mode, arm_version)), 'compact');
    
    for i = 1 : 6
        set(theta_hdl(i), 'xdata', time_stamp_data(plot_data_range), 'YData', ...
            joint_angle_data(plot_data_range,i));
    end
    savefig(joint_angle_fig_hdl, fullfile(pwd, ...
        sprintf('results/joint_angle_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type,...
        tracking_mode, arm_version)), 'compact');
    
    for i = 1 : 6
        set(joint_velocity_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', joint_velocity_data(plot_data_range,i));
    end
    savefig(joint_velocity_fig_hdl, fullfile(pwd, ...
        sprintf('results/joint_velocity_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type, ...
        tracking_mode, arm_version)), 'compact');
    
    set(ik_time_hdl, 'xdata', time_stamp_data(plot_data_range), ...
        'YData', ik_time_data(plot_data_range));
    savefig(ik_time_fig_hdl, fullfile(pwd, ...
        sprintf('results/ik_time_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type, ...
        tracking_mode, arm_version)), 'compact');
    
    for i = 1 : 6
        set(twist_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', twist_data(plot_data_range,i));
    end
    savefig(twist_fig_hdl, fullfile(pwd, ...
        sprintf('results/twist_velocity_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type, ...
        tracking_mode, arm_version)), 'compact');
    
    savefig(robot_fig_hdl, fullfile(pwd, ...
        sprintf('results/robot_final_%s_%s_%s_%s.fig', solver_type, ...
        trajectory_type, ...
        tracking_mode, arm_version)), 'compact');
end

drawnow; 

% show process info
fprintf('Task completed, total data processed: %d\n', slerp_step_count);

end % end of the main function