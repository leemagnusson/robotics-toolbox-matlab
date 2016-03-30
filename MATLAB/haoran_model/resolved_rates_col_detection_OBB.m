%% Resolved rates collision detection
% This code runs resolved rates for single arm with desired task space
% input.
% by Haoran Yu 3/16/2016
%%
% init
clc
clear all
close all
load('URDF_info.mat')
load('point_clouds_all.mat');
load('VertexData_origin.mat');
load('point_boundary_all.mat');
% init base transformation for both arms
base_T1=eye(4);
base_T2=[RotationMatrix_rad(pi/2,[0;0;1]) [-0.8; -0.8; 0]; 0 0 0 1];

% init both arms for resolved rates
q_init1 = [0;0;0;0;0;pi/8;-pi/8;0;0;0;0];
q_init2 = [0;0;0;0;0;0;0;0;0;0;0];
v_eef1 = [0.0; 0.3; 0.0];
v_eef2 = [0.0; 0; 0.0];
dt = 0.01;
q1 = q_init1;
q_rcm1 = convert2rcm(q1);
q2 = q_init2;
q_rcm2 = convert2rcm(q2);

%% resolved rates
%
figure(1)
hold on
% view(62,28)
view(3,22)
axis equal
% camzoom(2)
collision_number = 0;
movie_index=1;
tic
for j=1:1
        cla;
    %% Arm1
    % kinematics
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm1,base_T1);
    % jacobian and resolved rates
    [J_rcm1,J_car1,J_all1] = calc_Jacobian_all(Arm_Kinematics1);
    J1 = J_rcm1(1:3,1:4);
    q_dot1 = pinv(J1) * v_eef1;
    q_dot_all1 = [0;0;0;0;0;q_dot1;0;0];
    q1 = q1 + q_dot_all1 *dt;
    q_rcm1 = convert2rcm(q1);
    % generate point clouds from vertex data
    for i = 1:18
        R = Arm_Kinematics1(i).Tran_matrix(1:3,1:3);
        d = Arm_Kinematics1(i).Tran_matrix(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran1(:,i) = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics1(i).color;
            if ismember(i,[7 8 9 10])
                for index = 1 : length(point_boundary{i})
                    point_boundary_tran1{i}(index,:) = (R * point_boundary{i}(index,:)' + d)'; % transformed point clouds stored separately
                end
                                plotminbox(point_boundary_tran1{i});
                                hold on
            end
            plotSTL(VertexData_tran1(:,i),rgba)
            hold on
        end
    end
    %% Arm 2
    % kinematics
    Arm_Kinematics2 = Arm_Kinematics(link_input,joint_input,q_rcm2,base_T2);
    % jacobian and resolved rates
    [J_rcm2,J_car2,J_all2] = calc_Jacobian_all(Arm_Kinematics2);
    J2 = J_rcm2(1:3,1:4);
    q_dot2 = pinv(J2) * v_eef2;
    q_dot_all2 = [0;0;0;0;0;q_dot2;0;0];
    q2 = q2 + q_dot_all2 *dt;
    q_rcm2 = convert2rcm(q2);
    % generate point clouds from vertex data
    for i = 1:18
        R = Arm_Kinematics2(i).Tran_matrix(1:3,1:3);
        d = Arm_Kinematics2(i).Tran_matrix(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran2(:,i) = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics2(i).color;
            if ismember(i,[7 8 9 10])
                for index = 1 : length(point_boundary{i})
                    point_boundary_tran2{i}(index,:) = (R * point_boundary{i}(index,:)' + d)'; % transformed point clouds stored separately
                end
                %                 plotminbox(point_boundary_tran2{i});
                %                 hold on
            end
            
            plotSTL(VertexData_tran2(:,i),rgba)
            hold on
        end
    end
    %% detect collision
    % detect collision based on point clouds
    
    collision = false;
    for index1 = 7:10
        if collision
            break;
        end
        for index2 = 7:10
            % detect points from arm2 inside arm1 volume
            collision = collision | Collision_detection_boxes(point_boundary_tran1{index1},point_boundary_tran2{index2});
        end
    end
    
    % show and store collision result
    title(['Collision =' num2str(collision)])
    if collision
        collision_number = collision_number +1;
        % store collision joint values
        q1_store(:,collision_number) = q_rcm1;
        q2_store(:,collision_number) = q_rcm2;
        %         plot3(p(1),p(2),p(3),'Marker','.','MarkerSize',15);
        %         hold on
    end
    draw_coordinate_system([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    hold on
    axis([ -1.1 0.3 -1.0 0.4 -0.1 0.7])
    light('Position',[1 3 2]);
    light('Position',[-3 -1 -3]);
    drawnow;
    %     F(movie_index) = getframe(gcf);
    %     movie_index = movie_index + 1;
end
toc