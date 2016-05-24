%% 
%%
clc
clear all
close all
load('arm_version_1.0.mat')
load('vertex_arm_origin_1.0.mat')
load('coupling_matrix.mat')
robot_object.transformation_base_=eye(4);
q=[0,0,0,0,0,0,0,0,0,0,0]';
robot_object.CalculateFK(q);
[jacobian_spherical,jacobian_cartesian,jacobian_all] = robot_object.CalculateJacobianAll;
% [u,v,d] = svd(jacobian_all(1:3,:));
% force = u(:,3);
% torque_origin = jacobian_all(1:3,:)'*force;
torque_limit = [125 74 74 50 50 50 50]' * [-1 1];
torque_gravity = robot_object.InverseDynamics([0;0;9.81],zeros(11,1),zeros(11,1),'gravity');
torque_g = torque_gravity(1:7);
torque_limit(:,1) = torque_limit(:,1) + torque_g;
torque_limit(:,2) = torque_limit(:,2) + torque_g;
% torque_unscaled = torque_origin(1:7);
% for i = 1:7
%     if torque_origin(i)>0
%     scale(i) = torque_limit(i,2)/torque_origin(i);
%     else
%         scale(i) = torque_limit(i,1)/torque_origin(i);
%     end
% end
% scale_min = min(scale);
% torque_applied = torque_unscaled * scale_min;
% % torque_applied(2) = -50;
% % torque_applied(7) = 50;
% force_tip = pinv(jacobian_all(1:3,:)') * [torque_applied;0;0;0;0];
% norm(force_tip)
max_force_norm = 0;
for t1 = torque_limit(1,:)
    for t2 = torque_limit(2,:)
        for t3 = torque_limit(3,:)
            for t4 = torque_limit(4,:)
                for t5 = torque_limit(5,:)
                    for t6 = torque_limit(6,:)
                        for t7 = torque_limit(7,:)
                            torque_cur = [t1;t2;t3;t4;t5;t6;t7];
                            force_cur = pinv(jacobian_all(1:3,:)') * [torque_cur;0;0;0;0];
                            if norm(force_cur) > max_force_norm
                                max_force = force_cur;
                                max_force_norm = norm(max_force);
                                current_torque = torque_cur;
                            end
                        end
                    end
                end
            end
        end
    end
end