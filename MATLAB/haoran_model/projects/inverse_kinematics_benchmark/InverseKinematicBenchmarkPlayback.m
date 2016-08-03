function InverseKinematicBenchmarkPlayback(theta_init, tip_joint_index, ...
    solver_type, file_name, tracking_mode, varargin)
% function TestIK_gauss_seidel(q_init, base_transformation, ...
%    tip_joint_index, total_time)
% This function implements an inverse kinematic solution with the
% consideration of singularity, joint limits, and joint velocity limits
% and possibly joint acceleration limits

close all

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

% load data, first robot is right hand
disp('Loading data file ...');
[angular_velocity_in, linear_velocity_in, time_stamp, xyz_in, quaternion_in] = ...
    ik_obj.load_data(fullfile(pwd, sprintf('data/%s',file_name)), 1, 0); %#ok<ASGLU>
disp('Loading data file done');

robot_object.transformation_base_ = eye(4);
robot_object.CalculateFK(theta_init);

% scale the translation coordinates
for i = 1: length(robot_object.frames_)
     robot_object.frames_(1:3, 4, i) = robot_object.frames_(1:3, 4, i) * ...
         ik_obj.length_unit_scale_;
     robot_object.frames_in_parent_(1:3, 4, i) = ...
         robot_object.frames_in_parent_(1:3, 4, i) * ...
         ik_obj.length_unit_scale_;
end

pose_init = robot_object.frames_(:, :, tip_joint_index);
pose_target = pose_init;
theta_start = theta_init;
joint_angles_prev = theta_init;
pose_current = pose_init;
pose_command = pose_init;
joint_velocities_prev = zeros(6,1); %#ok<NASGU>

if (enable_data_log)
    orientation_fig_hdl = figure;
    time_stamp_data= [0, 0];
    orientation_error_data = [zeros(1,3);zeros(1,3)];
    orientation_error_hdl = plot(time_stamp_data, orientation_error_data);
    legend('roll', 'pitch', 'yaw');
    title(sprintf('Orientation error-%s-%s',solver_type, tracking_mode));
    ylabel('Orientation error(deg)');
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
    legend('1', '2', '3', '4', '5', '6');
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
axis_range = [-0.08 0.08 -0.38 -0.20 0.20 0.35];
ik_obj.DrawRobot(axis_range);

DrawCoordinateSystem([1.0 1.0 1.0] * 0.005 * ...
    ik_obj.length_unit_scale_, pose_init(1:3, 1:3), ...
    pose_init(1:3, 4),'rgb');
title(sprintf('Robot EE trajectory-%s-%s',solver_type, tracking_mode));
light('Position',[1 3 -3]);
light('Position',[1 -3 3]);
view(40 , 10);
ax = gca;
ax.Color = [0.98, 1, 1]; % lightcyan
ax.Box = 'on';
ax.BoxStyle = 'full';
ax.LineWidth = 0.1;
drawnow;

% iterate until converge
i_start = 1;
% i_end = length(time_stamp) - 1;
%  i_start = 3000;
 i_end = 4000;

play_back_frames(1: i_end - i_start + 1) = struct('cdata',[], 'colormap',[]);

% display processing start
fprintf('Processing: %09d', i_start)

for i_step = i_start: i_end  

    % update processing count
    fprintf('\b\b\b\b\b\b\b\b\b\b %09d', i_step - i_start);
    
    if(enable_data_log)
        % save the current frame for later use
        play_back_frames(i_step - i_start + 1) = getframe;
    end
    
    % update joint angles
    theta_start = joint_angles_prev;
    pose_prev = pose_current;
    
    % compute commanded pose base on tracking mode
    switch tracking_mode
        case 'absolute'
            pose_command(1:3,4) = pose_init(1:3,4) + ...
                ((xyz_in(i_step + 1, :) - xyz_in(i_start, :))') * ...
                ik_obj.length_unit_scale_;
            rotation_command =  ...
                ik_obj.quaternion_to_rotation_wxyz(quaternion_in(i_step + 1,:)) * ...
                ik_obj.quaternion_to_rotation_wxyz(quaternion_in(i_start,:))' * ...
                pose_init(1:3, 1:3);
        case 'relative'
            pose_command(1:3,4) = pose_current(1:3,4) + ...
                ((xyz_in(i_step + 1, :) - xyz_in(i_step, :))') * ...
                ik_obj.length_unit_scale_;
            rotation_command =  ...
                ik_obj.quaternion_to_rotation_wxyz(quaternion_in(i_step + 1,:)) *...
                ik_obj.quaternion_to_rotation_wxyz(quaternion_in(i_step,:))' * ...
                pose_current(1:3, 1:3);
        otherwise
            error('Invalid tracking mode input: %s ', tracking_mode);
    end
    
    % calculate twist from current and command pose
    dt = time_stamp(i_step + 1) - time_stamp(i_step);
    if(dt ~= 0)
        
        % start timer
        tic
        % compute inverse kinematics        
        [joint_angles, joint_velocities, twist, pose_current, ik_iteration_steps] = ...
            ik_obj.compute_inverse_kinematics(pose_command(1:3, 4), ...
            rotation_command, dt, joint_angles_prev, ...
            theta_start, tip_joint_index);  %#ok<ASGLU>
        process_time = toc;
        
        % save joint angle and velocity for next step
        joint_angles_prev = joint_angles;

        if enable_data_log
            % store data into data array
            time_stamp_data = [time_stamp_data, time_stamp(i_step)]; %#ok<*AGROW>
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
    end
    % update robot plot
    ik_obj.DrawRobot(axis_range);
    line([pose_prev(1,4), pose_current(1,4), ], ...
        [pose_prev(2,4), pose_current(2,4)], ...
        [pose_prev(3,4), pose_current(3,4)], 'LineStyle','-.');
    drawnow;
end

if( enable_data_log)
    fprintf('\nWriting vedio file...');
    
    % create a vedio object
    video_writer_obj = VideoWriter(fullfile(pwd, ...
        sprintf('results/%s_%s_%s_%s.mp4', solver_type, file_name, tracking_mode,arm_version)));
    % write frames to the video writer
    open(video_writer_obj);
    for i = 1:length(play_back_frames)
        writeVideo(video_writer_obj, play_back_frames(i));
    end
    close(video_writer_obj);
    
    % plot the final result and save as figure
    plot_data_range = 3:length(time_stamp_data);
    
    for i = 1: 3
        set(orientation_error_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', orientation_error_data(plot_data_range,i));
    end
    savefig(orientation_fig_hdl, fullfile(pwd, ...
        sprintf('results/orientation_error_%s_%s_%d_%d_%s_%s.fig', solver_type, ...
        file_name, i_start, i_end, tracking_mode, arm_version)), 'compact');
    
    for i = 1:3
        set(position_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', pose_data(plot_data_range,i));
    end
    savefig(position_fig_hdl, fullfile(pwd, ...
        sprintf('results/position_error_%s_%s_%d_%d_%s_%s.fig', solver_type, ...
        file_name, i_start, i_end, tracking_mode, arm_version)), 'compact');
    
    for i = 1 : 6
        set(theta_hdl(i), 'xdata', time_stamp_data(plot_data_range), 'YData', ...
            joint_angle_data(plot_data_range,i));
    end
    savefig(joint_angle_fig_hdl, fullfile(pwd, ...
        sprintf('results/joint_angle_%s_%s_%d_%d_%s_%s.fig', solver_type, file_name,...
        i_start, i_end, tracking_mode, arm_version)), 'compact');
    
    for i = 1 : 6
        set(joint_velocity_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', joint_velocity_data(plot_data_range,i));
    end
    savefig(joint_velocity_fig_hdl, fullfile(pwd, ...
        sprintf('results/joint_velocity_%s_%s_%d_%d_%s_%s.fig', solver_type, ...
        file_name, i_start, i_end, tracking_mode, arm_version)), 'compact');
    
    set(ik_time_hdl, 'xdata', time_stamp_data(plot_data_range), ...
        'YData', ik_time_data(plot_data_range));
    savefig(ik_time_fig_hdl, fullfile(pwd, ...
        sprintf('results/ik_time_%s_%s_%d_%d_%s_%s.fig', solver_type, ...
        file_name, i_start, i_end, tracking_mode, arm_version)), 'compact');
    
    for i = 1 : 6
        set(twist_hdl(i), 'xdata', time_stamp_data(plot_data_range), ...
            'YData', twist_data(plot_data_range,i));
    end
    savefig(twist_fig_hdl, fullfile(pwd, ...
        sprintf('results/joint_velocity_%s_%s_%s_%s.fig', solver_type, ...
        file_name, ...
        tracking_mode, arm_version)), 'compact');
    
    savefig(robot_fig_hdl, fullfile(pwd, ...
        sprintf('results/robot_final_%s_%s_%s_%s.fig', solver_type, ...
        file_name, ...
        tracking_mode, arm_version)), 'compact');
    drawnow;
end

% show process info
fprintf('Task completed, total data processed: %d\n', i_end -i_start + 1);

end % end of the main function