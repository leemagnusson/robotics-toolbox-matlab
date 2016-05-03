%% Hernia procedure arm setup
% This code runs three arms with init clinial input of joint setup and
% automatically setup the hernia procedure. The clinical input is loaded
% from InitHerniaSetup.m. The file "q_init_setup_hernia.mat" will load the
% table adapter angles and selected table adapter number into variable
% "q_bed_adapter" and "selected_bed_adapter".
% This version of procedure setup uses the position and orientation on the
% trocar to align the cartesian arm first and then target the end-effector
% to the target on hernia body. The first procedure uses the 6DOF cartesian
% arm jacobian with includes joint 1 to 6. The second task is only position
% task and uses the 3 by 6 translation jacobian of the spherical arm.
%%
% init
clc
clear all
close all
load('q_init_setup_hernia.mat');
load('urdf_info.mat')
load('vertex_arm_origin.mat')
load('vertex_hernia_patient_body.mat');
load('arm_version_1.0.mat')
load('index_joints.mat');
load('coupling_matrix.mat')
load('vertex_bed.mat');
load('vertex_bed_adapter.mat');
% init hernia setup
InitHerniaSetup
% init resolved rates
InitIKParameters;

% init figure
fig_handle = figure(1);
set(fig_handle,'Position',[600 10 750 750])
hold on
view(3)
view(-6,43)
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
index_robot = 0;
% bed adapter
% q_bed_adapter = [0 -0.02 0;-40*pi/180 -0.18 0;0 -0.02 0]';
% selected_bed_adapter = [2 1 5];
for index_bed_adapter = selected_bed_adapter;
    % setup bed adapter
    index_robot = index_robot + 1;
    frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
    transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    
    %% resolved rates
    p_err = p_t(:,index_robot) - p_rcm(:,index_robot);
    rotation_err = rotation_t(:,:,index_robot) * rotation_rcm(:,:,index_robot)';
    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
    error_signal = 0;
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
            frames_bed_adapter1 = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,selected_bed_adapter(1)));
            frames_bed_adapter2 = CalculateBedAdapterFK(q_bed_adapter(:,2),frames_bed_adapter_base(:,:,selected_bed_adapter(2)));
            frames_bed_adapter3 = CalculateBedAdapterFK(q_bed_adapter(:,3),frames_bed_adapter_base(:,:,selected_bed_adapter(3)));
            DrawBed(vertex_bed,[0.0 0.7 0.0 1])
            hold on
            DrawBedAdapter(frames_bed_adapter1,vertex_bed_adapter,[1 0 0 1])
            hold on
            DrawBedAdapter(frames_bed_adapter2,vertex_bed_adapter,[1 0 0 1])
            hold on
            DrawBedAdapter(frames_bed_adapter3,vertex_bed_adapter,[1 0 0 1])
            hold on
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
            axis([-1.2 1 -0.8 0.8 -0.2 2.8])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
            index_movie = index_movie + 1;
            movie_frames(index_movie) = getframe(gcf);
        end
        error_signal = error_signal + 1;
        if error_signal >= 10000
            error('IK not converging!')
        end
    end
end
index_robot = 0;
for index_bed_adapter = [2 1 5];
    index_robot = index_robot + 1;
    frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
    transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    
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
    error_signal = 0;
    while((norm(p_err) > eps_translation)) % This is a position task to target the end-effector to the target
        cla
        frames_cur = robot_kinematics.CalculateFK(q_rcm(:,index_robot),transformation_base(:,:,index_robot));
        p_eef(:,index_robot) = frames_cur(1:3,4,index_eef);
        [twist_eef,p_err,theta_err] = ComputeTwist(p_t(:,index_robot),p_eef(:,index_robot),rotation_t(:,:,index_robot),rotation_eef(:,:,index_robot));
        
        [jacobian_spherical,jacobian_car,jacobian_all] = CalculateJacobianAll(frames_cur);
        
        jacobian = jacobian_spherical(1:3,1:3);
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
            frames_bed_adapter1 = CalculateBedAdapterFK(q_bed_adapter(:,1),frames_bed_adapter_base(:,:,selected_bed_adapter(1)));
            frames_bed_adapter2 = CalculateBedAdapterFK(q_bed_adapter(:,2),frames_bed_adapter_base(:,:,selected_bed_adapter(2)));
            frames_bed_adapter3 = CalculateBedAdapterFK(q_bed_adapter(:,3),frames_bed_adapter_base(:,:,selected_bed_adapter(3)));
            DrawBed(vertex_bed,[0.0 0.7 0.0 1])
            hold on
            DrawBedAdapter(frames_bed_adapter1,vertex_bed_adapter,[1 0 0 1])
            hold on
            DrawBedAdapter(frames_bed_adapter2,vertex_bed_adapter,[1 0 0 1])
            hold on
            DrawBedAdapter(frames_bed_adapter3,vertex_bed_adapter,[1 0 0 1])
            hold on
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
            axis([-1.2 1 -0.8 0.8 -0.2 2.8])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
            index_movie = index_movie + 1;
            movie_frames(index_movie) = getframe(gcf);
        end
        if error_signal > 10000
            error('IK not converging!');
        end
    end
    q_init_setup(:,index_robot) = WrapToPi(q(:,index_robot));
end

% save('data/q_init_setup_hernia.mat','q_init_setup','q_bed_adapter','selected_bed_adapter');