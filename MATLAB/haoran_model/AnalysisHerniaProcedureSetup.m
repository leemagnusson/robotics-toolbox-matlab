%% Hernia procedure arm setup
% This code runs three arms with init clinial input of joint setup and
% automatically setup the hernia procedure
%%
% init
clc
clear all
close all

load('urdf_info.mat')
load('vertex_arm_origin.mat')
load('vertex_hernia_patient_body.mat');
load('arm_version_1.0.mat')
load('index_joints.mat');
load('coupling_matrix.mat')

% init hernia setup
InitHerniaSetup
% init resolved rates
InitIKParameters;

% init figure
fig_handle = figure(1);
set(fig_handle,'Position',[600 10 750 750])
hold on
view(3)
view(0,90)
% camzoom(1.2)
% camzoom(5)
axis equal
index_movie = 0;
arm_color = GetRobotColor(robot_kinematics);
do_plot = 0;
% initialize all the arms
for index_robot =1:3
    q(:,index_robot) = q_set(:,index_robot);
    q_rcm(:,index_robot) = ConvertToRcm(q(:,index_robot),coupling_matrix);
    frames_init = robot_kinematics.CalculateFK(q_rcm(:,index_robot),transformation_base(:,:,index_robot));
    p_rcm(:,index_robot) = frames_init(1:3,4,index_rcm);
    rotation_rcm(:,:,index_robot) = frames_init(1:3,1:3,index_car);
    p_t(:,index_robot) = translation_trocar(:,index_robot); % target position
    rotation_t(:,:,index_robot) = rotation_trocar(:,:,index_robot); % target orientation
end

% Use 6Dof cartesian to move the rcm link
converged = zeros(3,1);
for index_robot = 1:3
    %% resolved rates
    p_err = p_t(:,index_robot) - p_rcm(:,index_robot);
    rotation_err = rotation_t(:,:,index_robot) * rotation_rcm(:,:,index_robot)';
    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
    while((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation))
        cla
        frames_cur = robot_kinematics.CalculateFK(q_rcm(:,index_robot),transformation_base(:,:,index_robot));
        p_rcm(:,index_robot) = frames_cur(1:3,4,index_rcm);
        rotation_rcm(:,:,index_robot) = frames_cur(1:3,1:3,index_car);
        [twist_rcm,p_err,theta_err] = ComputeTwist(p_t(:,index_robot),p_rcm(:,index_robot),rotation_t(:,:,index_robot),rotation_rcm(:,:,index_robot));
        [jacobian_6dof] = CalculateJacobian6DofRCM(frames_cur);
        q_dot = pinv(jacobian_6dof)*twist_rcm;
        q_dot_all = [q_dot;0;0;0;0;0];
        % update q
        q(:,index_robot) = q(:,index_robot) + q_dot_all *dt;
        q_rcm(:,index_robot) = ConvertToRcm(q(:,index_robot),coupling_matrix);
        
        if do_plot
            % Draw robot
            frames1 = robot_kinematics.CalculateFK(q_rcm(:,1),transformation_base(:,:,1));
            frames2 = robot_kinematics.CalculateFK(q_rcm(:,2),transformation_base(:,:,2));
            frames3 = robot_kinematics.CalculateFK(q_rcm(:,3),transformation_base(:,:,3));
            DrawRobotNoTool(frames1,vertex_arm_origin,arm_color)
            hold on
            DrawRobotNoTool(frames2,vertex_arm_origin,arm_color)
            hold on
            DrawRobotNoTool(frames3,vertex_arm_origin,arm_color)
            hold on
            DrawCoordinateSystem([0.02 0.02 0.02],rotation_t(:,:,index_robot),p_t(:,index_robot),'rgb')
            hold on
            DrawCoordinateSystem([0.02 0.02 0.02],frames_cur(1:3,1:3,index_car),frames_cur(1:3,4,index_rcm),'rgb')
            hold on
            % Draw hernia
            vertex_hernia_patient_body_transformed = transformSTL(vertex_hernia_patient_body,rotation_hernia_patient,translation_hernia_patient);
            rgba = [0 0 1 0.1];
            PlotStl(vertex_hernia_patient_body_transformed,rgba);
            hold on
            axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
            index_movie = index_movie + 1;
            movie_frames(index_movie) = getframe(gcf);
        end
    end
end
for index_robot =1:3
    % set rcm pose and target pose
    q_rcm(:,index_robot) = ConvertToRcm(q(:,index_robot),coupling_matrix);
    frames_init = robot_kinematics.CalculateFK(q_rcm(:,index_robot),transformation_base(:,:,index_robot));
    p_eef(:,index_robot) = frames_init(1:3,4,index_eef);
    rotation_eef(:,:,index_robot) = frames_init(1:3,1:3,index_eef);
    if index_robot==2
        p_t(:,index_robot) = translation_camera;
    else
        p_t(:,index_robot) = 0.9 * translation_hernia + 0.1 * translation_trocar(:,index_robot); % target position
    end
    rotation_t(:,:,index_robot) = rotation_eef(:,:,index_robot); % target orientation
    % resolved rates
    p_err = p_t(:,index_robot) - p_eef(:,index_robot);
    while((norm(p_err) > eps_translation))
        cla
        frames_cur = robot_kinematics.CalculateFK(q_rcm(:,index_robot),transformation_base(:,:,index_robot));
        p_eef(:,index_robot) = frames_cur(1:3,4,index_eef);
        [twist_eef,p_err,theta_err] = ComputeTwist(p_t(:,index_robot),p_eef(:,index_robot),rotation_t(:,:,index_robot),rotation_eef(:,:,index_robot));
        
        [jacobian_rcm,jacobian_car,jacobian_all] = CalculateJacobianAll(frames_cur);
        
        jacobian = jacobian_rcm(1:3,1:3);
        v_eef = twist_eef(1:3);
        q_dot = pinv(jacobian)*v_eef;
        q_dot_all = [0;0;0;0;0;q_dot;0;0;0];
        % update q
        q(:,index_robot) = q(:,index_robot) + q_dot_all *dt;
        q_rcm(:,index_robot) = ConvertToRcm(q(:,index_robot),coupling_matrix);
        
        if do_plot
            % Draw robot
            frames1 = robot_kinematics.CalculateFK(q_rcm(:,1),transformation_base(:,:,1));
            frames2 = robot_kinematics.CalculateFK(q_rcm(:,2),transformation_base(:,:,2));
            frames3 = robot_kinematics.CalculateFK(q_rcm(:,3),transformation_base(:,:,3));
            DrawRobot(frames1,vertex_arm_origin,arm_color)
            hold on
            DrawRobot(frames2,vertex_arm_origin,arm_color)
            hold on
            DrawRobot(frames3,vertex_arm_origin,arm_color)
            hold on
            % Draw hernia
            vertex_hernia_patient_body_transformed = transformSTL(vertex_hernia_patient_body,rotation_hernia_patient,translation_hernia_patient);
            rgba = [0 0 1 0.1];
            PlotStl(vertex_hernia_patient_body_transformed,rgba);
            hold on
            axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
            index_movie = index_movie + 1;
            movie_frames(index_movie) = getframe(gcf);
        end
    end
    q_init_setup(:,index_robot) = WrapToPi(q(:,index_robot));
end

% save('data/q_init_setup_hernia2.mat','q_init_setup');