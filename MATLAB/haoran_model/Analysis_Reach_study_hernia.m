clc
clear all
close all
load('URDF_info.mat');
load('VertexData_origin.mat');
load('VertexData_Hernia_Body.mat');
% load('q_init_setup.mat');
load('q_init_setup_hernia_best.mat');
load('Tool_path.mat');
load('point_boundary_all.mat');
init_Hernia_setup
dt = 0.0025;
ncycle_t = 5;
ncycle_r = 2;
v_max = 0.25;
omega_max = 10.5;
p_eps = 0.001;
theta_eps = 1 * pi/180;

fig_handle = figure(1);
set(fig_handle,'Position',[600 10 750 750])
hold on
axis equal
view(3)
view(-2,43)
q(:,1) = q_init_setup(:,1);
q_rcm(:,1) = convert2rcm(q_init_setup(:,1));
Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm(:,1),base_T_setup(:,:,1));
p_eef(:,1) = Arm_Kinematics1(14).Tran_matrix(1:3,4);
R_eef(:,:,1) = Arm_Kinematics1(14).Tran_matrix(1:3,1:3);

q(:,2) = q_init_setup(:,2);
q_rcm(:,2) = convert2rcm(q_init_setup(:,2));

q(:,3) = q_init_setup(:,3);
q_rcm(:,3) = convert2rcm(q_init_setup(:,3));
Arm_Kinematics3 = Arm_Kinematics(link_input,joint_input,q_rcm(:,3),base_T_setup(:,:,3));
p_eef(:,3) = Arm_Kinematics3(14).Tran_matrix(1:3,4);
R_eef(:,:,3) = Arm_Kinematics3(14).Tran_matrix(1:3,1:3);

% for j = 1 : length(Tool_path_left)
%     Tool_path_left(1:3,4,j) = R_Camera * Tool_path_left(1:3,4,j) + p_Camera;
%     Tool_path_left(1:3,1:3,j) = R_Camera * Tool_path_left(1:3,1:3,j);
%     Tool_path_right(1:3,4,j) = R_Camera * Tool_path_right(1:3,4,j) + p_Camera;
%     Tool_path_right(1:3,1:3,j) = R_Camera * Tool_path_right(1:3,1:3,j);
%     plot3(Tool_path_left(1,4,j),Tool_path_left(2,4,j),Tool_path_left(3,4,j),'Color','r','Marker','o')
%     hold on
%     plot3(Tool_path_right(1,4,j),Tool_path_right(2,4,j),Tool_path_right(3,4,j),'Color','g','Marker','o')
%     hold on
% end
movie_index = 1;
number_col = 1;
num_q = 1;
index_start = 1767;
index_end = 2592;
sample_rate = 1;
for j = index_start :sample_rate: index_end
    %     Tool_path_left(1:3,4,j) = R_Camera * Tool_path_left(1:3,4,j) + p_Camera;
    Tool_path_left(1:3,4,j) = R_Camera * Tool_path_left(1:3,4,1) + p_Camera + [-0.00025*(j-1767);0;0];
    Tool_path_left(1:3,1:3,j) = R_Camera * Tool_path_left(1:3,1:3,j);
    Tool_path_right(1:3,4,j) = R_Camera * Tool_path_right(1:3,4,j) + p_Camera;
    Tool_path_right(1:3,1:3,j) = R_Camera * Tool_path_right(1:3,1:3,j);
    
    p_t(:,1) = Tool_path_left(1:3,4,j);
    p_t(:,3) = Tool_path_right(1:3,4,j);
    
    R_t(:,:,1) = Tool_path_left(1:3,1:3,j);
    R_t(:,:,3) = Tool_path_right(1:3,1:3,j);
    
    for index = [1 3]
        p_err = p_t(:,index) - p_eef(:,index);
        R_err = R_t(:,:,index) * R_eef(:,:,index)';
        theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
        
        while((norm(p_err) > p_eps) || (abs(theta_err) > theta_eps))
            Arm_Kinematics_cur = Arm_Kinematics(link_input,joint_input,q_rcm(:,index),base_T_setup(:,:,index));
            p_eef(:,index) = Arm_Kinematics_cur(14).Tran_matrix(1:3,4);
            R_eef(:,:,index) = Arm_Kinematics_cur(14).Tran_matrix(1:3,1:3);
            p_err = p_t(:,index) - p_eef(:,index);
            if norm(p_err) < p_eps
                v_eef = zeros(3,1);
            elseif norm(p_err) < ncycle_t * v_max * dt
                v_eef = p_err/(ncycle_t*dt);
            else
                v_eef = v_max * p_err/norm(p_err);
            end
            
            R_err = R_t(:,:,index) * R_eef(:,:,index)';
            theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
            vect_err = 1/(2*sin(theta_err))*[(R_err(3,2)-R_err(2,3));(R_err(1,3)-R_err(3,1));(R_err(2,1)-R_err(1,2))];
            
            if abs(theta_err) < theta_eps
                omega_eef = zeros(3,1);
            elseif abs(theta_err) < ncycle_r * omega_max * dt
                omega_eef = theta_err * vect_err/(ncycle_r*dt);
            else
                omega_eef = omega_max * vect_err;
            end
            t_eef = [v_eef;omega_eef];
            [J_rcm,J_car,J_all] = calc_Jacobian_all(Arm_Kinematics_cur);
            q_dot = pinv(J_rcm)*t_eef;
            q_dot_all = [0;0;0;0;0;q_dot];
            % update q
            q(:,index) = q(:,index) + q_dot_all *dt;
            q_rcm(:,index) = convert2rcm(q(:,index));
        end
    end
    
    q_store(:,:,num_q) = q;
    num_q = num_q + 1;
    
    for index = 1:3
        % generate point clouds from vertex data
        Arm_Kinematics_cur = Arm_Kinematics(link_input,joint_input,q_rcm(:,index),base_T_setup(:,:,index));
        for i = 7:10
            R = Arm_Kinematics_cur(i).Tran_matrix(1:3,1:3);
            d = Arm_Kinematics_cur(i).Tran_matrix(1:3,4);
            for index_boundary = 1 : length(point_boundary{i})
                point_boundary_tran{i,index}(index_boundary,:) = (R * point_boundary{i}(index_boundary,:)' + d)'; % transformed point clouds stored separately
            end
            %              plotminbox(point_boundary_tran{i,index});
            %              hold on
        end
    end
    
    [Collision12,collide_index12] = find_collision_two_arm (point_boundary_tran(:,1),point_boundary_tran(:,2),7:10);
    [Collision13,collide_index13] = find_collision_two_arm (point_boundary_tran(:,1),point_boundary_tran(:,3),7:10);
    [Collision23,collide_index23] = find_collision_two_arm (point_boundary_tran(:,2),point_boundary_tran(:,3),7:10);

    Arm_Kinematics_cur1 = Arm_Kinematics(link_input,joint_input,q_rcm(:,1),base_T_setup(:,:,1));
    Arm_Kinematics_cur2 = Arm_Kinematics(link_input,joint_input,q_rcm(:,2),base_T_setup(:,:,2));
    Arm_Kinematics_cur3 = Arm_Kinematics(link_input,joint_input,q_rcm(:,3),base_T_setup(:,:,3));
    
    [Arm_Kinematics_cur1,Arm_Kinematics_cur2] = Change_link_color(Arm_Kinematics_cur1,Arm_Kinematics_cur2,Collision12,collide_index12);
    [Arm_Kinematics_cur1,Arm_Kinematics_cur3] = Change_link_color(Arm_Kinematics_cur1,Arm_Kinematics_cur3,Collision13,collide_index13);
    [Arm_Kinematics_cur2,Arm_Kinematics_cur3] = Change_link_color(Arm_Kinematics_cur2,Arm_Kinematics_cur3,Collision23,collide_index23);
    
%     cla
    %     draw_coordinate_system([0.02 0.02 0.02],R_Camera,p_Camera,'rgb','c')
    %     hold on
    %     draw_coordinate_system([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
    %     hold on
    
%     Draw_Robot_Arm(Arm_Kinematics_cur1,VertexData_origin)
%     hold on
%     Draw_Robot_Arm(Arm_Kinematics_cur2,VertexData_origin)
%     hold on
%     Draw_Robot_Arm(Arm_Kinematics_cur3,VertexData_origin)
%     hold on
    
    if Collision12 || Collision13 || Collision23
        q_store_collision(:,:,number_col) = q;
        Collision_store(:,number_col) = [Collision12;Collision13;Collision23];
%         title(['Collision12 =' num2str(Collision12) ';   ' 'Collision13 =' num2str(Collision13) ';   ' 'Collision23 =' num2str(Collision23)],'Color','r')
        number_col = number_col + 1;
    else
%         title(['Collision12 =' num2str(Collision12) ';   ' 'Collision13 =' num2str(Collision13) ';   ' 'Collision23 =' num2str(Collision23)],'Color','b')
    end
    
%     VertexData_hernia_tran = transformSTL(VertexData_Hernia_Body,R_Hernia_Body,Hernia_Body);
%     rgba = [0 0 1 0.1];
%     plotSTL(VertexData_hernia_tran,rgba);
%     hold on
%     axis([-0.45 0.6 -0.5 0.8 -0.2 0.65])
%     light('Position',[1 3 2]);
%     light('Position',[-3 -1 -3]);
%     drawnow;
    %     F(movie_index) = getframe(gcf);
    %     movie_index = movie_index + 1;
end