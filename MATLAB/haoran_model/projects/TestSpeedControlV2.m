%% test the speed regulation method 2
% This code test the speed regulation algorithm method 2. The method could
% be found from presentation on google drive. The document has not been
% written.
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
q_dot_max = [1.5 1.5 0.16 2.5 2.5 2.5]';
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
%     if mod(index2,2) == 0
%         p_t = p_t_init + [-0.2;0;0.02*index2];
%     else
%         p_t = p_t_init + [0;0;0.02*index2];
%     end
        p_t = -0.15 * robot_object.frames_(1:3,1,14) + p_eef;
    p_t_1 = p_eef;
    p_err = p_t - p_eef;
    rotation_err = rotation_t * rotation_eef';
    theta_err = acos((rotation_err(1,1)+rotation_err(2,2)+rotation_err(3,3)-1)/2);
    
    %     plot3(p_t(1),p_t(2),p_t(3),'Marker','*','MarkerSize',10);
    %     hold on
    while((norm(p_err) > eps_translation) || (abs(theta_err) > eps_rotation))
        %         cla
        
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
        min_linear_axis = u1(:,3);
        min_angular_axis = u2(:,3);
        q_dot_max_cur = pinv(jacobian)*[v_max *min_linear_axis ; omega_max *min_linear_axis];
        scale_down_cur = abs(q_dot_max_cur)./q_dot_max;
        scale_down = max(scale_down_cur);
        
        q_dot = pinv(jacobian)*twist_eef;
        %         f_scale = -0.5*tanh(1*(scale-1)) + 0.5;
%         f_scale = -0.125*tanh(1*(scale_down-1)) + 0.875;
                f_scale = 1;
        if scale_down > 1;
            twist_actual = f_scale * twist_eef / scale_down;
        else
            twist_actual = f_scale * twist_eef;
        end
        
        q_dot_actual = pinv(jacobian)*twist_actual;
        
        % update q
        q_dot_all = [0;0;0;0;0;q_dot_actual];
        q = q + q_dot_all * dt;
        index = index + 1;
        scale_store(index) = scale_down;
        condition_number_store(index) = e(1,1)/e(end,end);
        q_store(:,index) = q;
        qd_store(:,index) = q_dot;
        qd_actual_store(:,index) = q_dot_actual;
        t_store(:,index) = twist_eef;
        t_actual_store(:,index) = twist_actual;
        eef_store(index) = norm(p_eef - p_t_1);
    end
end
for index3 = 1:length(t_store)
    velocity_eef(index3) = norm(t_store(1:3,index3));
    velocity_actual_eef(index3) = norm(t_actual_store(1:3,index3));
    angular_velocity_eef(index3) = norm(t_store(4:6,index3));
    angular_velocity_actual_eef(index3) = norm(t_actual_store(4:6,index3));
    diff_twist_direction(index3) = norm(t_store(:,index3)/norm(t_store(:,index3)) - t_actual_store(:,index3)/norm(t_actual_store(:,index3)));
end
robot_object.DrawRobot(vertex_arm_origin,[14],0.1)
hold on
axis([-0.8 0.8 -1.2 0.3 -0.3 0.9])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);

drawnow;
figure(2)
plot(1:length(t_store),[qd_store(:,:);qd_actual_store(:,:)])
% plot(eef_store,[qd_store(:,:);qd_actual_store(:,:)])
legend('6','7','8','9','10','11','6r','7r','8r','9r','10r','11r');
figure(3)
plot(1:length(t_store),[velocity_eef;velocity_actual_eef])
% plot(eef_store,[velocity_eef;velocity_actual_eef])
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