%% Hernia procedure arm setup
% This code runs three arms with init clinial input of joint setup and
% automatically setup the hernia procedure. The clinical input is loaded
% from InitHerniaSetup.m. The file "q_init_setup_hernia.mat" will load the
% table adapter angles and selected table adapter number into variable
% "q_bed_adapter" and "selected_bed_adapter".
% This version of procedure setup uses the only  the position on the
% trocar to align the cartesian arm first and then target the end-effector
% to the target on hernia body. The first procedure uses the 3by6 cartesian
% arm jacobian which includes joint 1 to 6. The second task is only position
% task and uses the 3 by 6 translation jacobian of the spherical arm.
% The result will be saved in 'q_init_setup_hernia.mat'. This result may
% not gurantee the arm to be well separated. So one can perform arm
% repositioning by using the function 'ArmRepositioning.m'. The user
% interface Arm_Setup_UI.m has the functionality to automatically perform
% arm procedure setup and arm repositioning.
%%
% init
clc
clear all
close all
load('urdf_info_1.0.mat');
load('vertex_arm_origin_1.0.mat');
load('arm_version_1.0.mat');
load('vertex_patient_body.mat');
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
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
do_plot = 1;
index_robot = 0;
selected_bed_adapter = [2 1 5];
q_bed_adapter = [0 -0.02 0;-40*pi/180 -0.18 0;0 -0.02 0]';

% initialize all the arms
for index_robot =1:3
    
    frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
    transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    
    q(:,index_robot) = q_set(:,index_robot);
    robot_arms{index_robot}.transformation_base_ = transformation_base(:,:,index_robot);
    robot_arms{index_robot}.CalculateFK(q(:,index_robot));
    p_rcm(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_rcm);
    rotation_rcm(:,:,index_robot) = robot_arms{index_robot}.frames_(1:3,1:3,index_car);
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
    %% resolved rates
    p_err = p_t(:,index_robot) - p_rcm(:,index_robot);
    rotation_err = rotation_t(:,:,index_robot) * rotation_rcm(:,:,index_robot)';
    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
    if do_plot
        while((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation))
            %     while((norm(p_err) > eps_translation))
            cla
            robot_arms{index_robot}.CalculateFK(q(:,index_robot));
            p_rcm(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_rcm);
            rotation_rcm(:,:,index_robot) = robot_arms{index_robot}.frames_(1:3,1:3,index_car);
            [twist_rcm,p_err,theta_err] = ComputeTwist(p_t(:,index_robot),p_rcm(:,index_robot),rotation_t(:,:,index_robot),rotation_rcm(:,:,index_robot));
            [jacobian_6dof] = robot_arms{index_robot}.CalculateJacobian6DofRCM;
            q_dot = pinv(jacobian_6dof)*twist_rcm;
            %         q_dot = pinv(jacobian_6dof(1:3,:))*twist_rcm(1:3);
            q_dot_all = [q_dot;0;0;0;0;0];
            % update q
            q(:,index_robot) = q(:,index_robot) + q_dot_all *dt;
            
            
            % Draw robot
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
            robot_arms{1}.DrawRobotNoTool(vertex_arm_origin)
            hold on
            robot_arms{2}.DrawRobotNoTool(vertex_arm_origin)
            hold on
            robot_arms{3}.DrawRobotNoTool(vertex_arm_origin)
            hold on
            DrawCoordinateSystem([0.02 0.02 0.02],rotation_t(:,:,index_robot),p_t(:,index_robot),'rgb')
            hold on
            DrawCoordinateSystem([0.02 0.02 0.02],robot_arms{index_robot}.frames_(1:3,1:3,index_car),robot_arms{index_robot}.frames_(1:3,4,index_rcm),'rgb')
            hold on
            % Draw hernia
            vertex_patient_body_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
            rgba = [0 0 1 0.1];
            PlotStl(vertex_patient_body_transformed,rgba);
            hold on
            axis([-1.2 1 -0.8 0.8 -0.2 2.8])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
            index_movie = index_movie + 1;
            movie_frames(index_movie) = getframe(gcf);
        end
    else
        q(:,index_robot) = robot_arms{index_robot}.InverseKinematics(q(:,index_robot),p_t(:,index_robot),rotation_t(:,:,index_robot),'Cartesian 6');
    end
end
index_robot = 0;
for index_bed_adapter = selected_bed_adapter;
    index_robot = index_robot + 1;
    frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
    transformation_base(:,:,index_robot) = frames_bed_adapter(:,:,end);
    
    % set rcm pose and target pose
    robot_arms{index_robot}.CalculateFK(q(:,index_robot));
    p_eef(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_eef);
    rotation_eef(:,:,index_robot) = robot_arms{index_robot}.frames_(1:3,1:3,index_eef);
    if index_robot==2
        p_t(:,index_robot) = translation_camera;
    else
        p_t(:,index_robot) = 0.9 * translation_target + 0.1 * translation_trocar(:,index_robot); % target position
    end
    rotation_t(:,:,index_robot) = rotation_eef(:,:,index_robot); % target orientation
    % resolved rates
    p_err = p_t(:,index_robot) - p_eef(:,index_robot);
    if do_plot
        while((norm(p_err) > eps_translation))
            cla
            robot_arms{index_robot}.CalculateFK(q(:,index_robot));
            p_eef(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_eef);
            [twist_eef,p_err,theta_err] = ComputeTwist(p_t(:,index_robot),p_eef(:,index_robot),rotation_t(:,:,index_robot),rotation_eef(:,:,index_robot));
            
            [jacobian_spherical,jacobian_car,jacobian_all] = robot_arms{index_robot}.CalculateJacobianAll;
%             jacobian_car_6DoF = robot_arms{index_robot}.CalculateJacobian6DofRCM;
%             jacobian_repositioning = [jacobian_all(1:3,:);jacobian_car_6DoF(1:3,:) zeros(3,5)];
            
            v_eef = twist_eef(1:3);
%             q_dot = pinv(jacobian_repositioning)*[v_eef;0;0;0];
            q_dot = pinv(jacobian_spherical(1:3,:)) * v_eef;
            q_dot_all = [0;0;0;0;0;q_dot];
            % update q
            q(:,index_robot) = q(:,index_robot) + q_dot_all *dt;
 
            % Draw robot
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
            robot_arms{1}.DrawRobot(vertex_arm_origin)
            hold on
            robot_arms{2}.DrawRobot(vertex_arm_origin)
            hold on
            robot_arms{3}.DrawRobot(vertex_arm_origin)
            hold on
            % Draw hernia
            vertex_patient_body_transformed = transformSTL(vertex_patient_body,rotation_patient,translation_patient);
            rgba = [0 0 1 0.1];
            PlotStl(vertex_patient_body_transformed,rgba);
            hold on
            axis([-1.2 1 -0.8 0.8 -0.2 2.8])
            light('Position',[1 3 2]);
            light('Position',[-3 -1 -3]);
            drawnow;
            index_movie = index_movie + 1;
            movie_frames(index_movie) = getframe(gcf);
        end
    else
        q(:,index_robot) = robot_arms{index_robot}.InverseKinematics(q(:,index_robot),p_t(:,index_robot),rotation_t(:,:,index_robot),'Spherical 3');
    end
    q_init_setup(:,index_robot) = WrapToPi(q(:,index_robot));
end
q_init_setup = [q_init_setup,zeros(11,1)];
q_bed_adapter = [q_bed_adapter,zeros(3,1)];
selected_bed_adapter = [selected_bed_adapter,10];
% save('../data/q_init_setup_hernia.mat','q_init_setup','q_bed_adapter','selected_bed_adapter');