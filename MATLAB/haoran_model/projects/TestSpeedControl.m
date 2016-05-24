%% test the speed regulation method 1
% This code test the speed regulation algorithm method 1. The method could
% be found from presentation on google drive. The document has not been
% written. The method 1 is using the commanded eef twist for motion
% scaling.
% https://drive.google.com/drive/u/0/folders/0B_fDry3ewv7raGhkVjFYblpiOVk
%%
% init
clc
clear all
close all
load('urdf_info_1.0.mat')
load('vertex_arm_origin_1.0.mat')
load('arm_version_1.0.mat')
load('coupling_matrix.mat')
% init resolved rates
% sets the iterative inverse kinematics parameters
InitIKParameters;

robot_object.transformation_base_ = eye(4);
q_init = [0,0,0,0,0,0,0,-0.10,0,0,0]';
q = q_init;
robot_object.CalculateFK(q);
p_eef = robot_object.frames_(1:3,4,14); % eef position
rotation_eef = robot_object.frames_(1:3,1:3,14); % eef orientation
p_t_init = p_eef + [0.1;-0.15;0]; % target position
rotation_t = rotation_eef; % target orientation
q_dot_max = [1.5 1.5 0.16 1.5 1.5 1.5]';
%% resolved rates
figure(1)
hold on
view(49,16)
% camzoom(5)
axis equal
%         grid on
grid minor
index_movie = 0;
index = 0;
for index2 = 1 : 1
    if mod(index2,2) == 0
        p_t = p_t_init + [-0.2;0;0.02*index2];
    else
        p_t = p_t_init + [0;0;0.02*index2];
    end
    p_t = -0.15 * robot_object.frames_(1:3,1,14) + p_eef;
    p_err = p_t - p_eef;
    rotation_err = rotation_t * rotation_eef';
    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
    % iteration_steps is set to 1000 because this library is not for real-time
    % control. For real-time control the iteration_steps should be re-defined.
    iteration_steps = 0;
    while (((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation)) && iteration_steps<=1000)
        iteration_steps = iteration_steps + 1;
        robot_object.CalculateFK(q);
        p_eef = robot_object.frames_(1:3,4,14);
        plot3(p_eef(1),p_eef(2),p_eef(3),'Marker','.','MarkerSize',5,'color','r');
        hold on
        rotation_eef = robot_object.frames_(1:3,1:3,14);
        % compute twist
        [twist_eef,p_err,theta_err] = ComputeTwist(p_t,p_eef,rotation_t,rotation_eef);
        
        % get Jacobian
        [jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
        jacobian = jacobian_spherical;
        [u,e,v] = svd(jacobian);
        [u1,e1,v1] = svd(jacobian(1:3,:));
        [u2,e2,v2] = svd(jacobian(4:6,:));
        q_dot = pinv(jacobian)*twist_eef;
        
        scale_down_cur = abs(q_dot)./q_dot_max;
        scale_down = max(scale_down_cur);
        
        if scale_down > 1;
            q_dot_actual = q_dot/scale_down;
        else
            q_dot_actual = q_dot;
        end
        twist_actual = jacobian * q_dot_actual;
        % update q
        q_dot_all = [0;0;0;0;0;q_dot_actual];
        q = q + q_dot_all *dt;
        index = index + 1;
        scale_store(index) = scale_down;
        condition_number_store(index) = e(1,1)/e(end,end);
        q_store(:,index) = q;
        qd_store(:,index) = q_dot;
        qd_actual_store(:,index) = q_dot_actual;
        t_store(:,index) = twist_eef;
        t_actual_store(:,index) = twist_actual;
    end
end
for index3 = 1:length(t_store)
    velocity_eef(index3) = norm(t_store(1:3,index3));
    velocity_actual_eef(index3) = norm(t_actual_store(1:3,index3));
    angular_velocity_eef(index3) = norm(t_store(4:6,index3));
    angular_velocity_actual_eef(index3) = norm(t_actual_store(4:6,index3));
    diff_twist_direction(index3) = norm(t_store(:,index3)/norm(t_store(:,index3)) - t_actual_store(:,index3)/norm(t_actual_store(:,index3)));
end
robot_object.DrawRobot(vertex_arm_origin,[],0.1)
hold on
axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);

drawnow;
figure(2)
plot(1:length(t_store),[qd_store([1:2,4:6],:);qd_actual_store([1:2,4:6],:)])
legend('6','7','9','10','11','6r','7r','9r','10r','11r');
figure(3)
plot(1:length(t_store),[velocity_eef;velocity_actual_eef])
legend('veef','veefr')

figure(4)
plot(1:length(t_store),[angular_velocity_eef;angular_velocity_actual_eef])
legend('weef','weefr')

figure(5)
plot(1:length(t_store),diff_twist_direction)
legend('difft')

figure(6)
plot(1:length(t_store),[condition_number_store/30;scale_store])
legend('cn','scale')