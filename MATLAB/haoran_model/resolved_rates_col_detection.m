clc
clear all
close all
% URDF_file= 'necessity_V1.URDF';
URDF_file= 'V1_Arm_URDF.URDF';
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;
load('point_clouds_all.mat');
base_T = eye(4);
A = [1 0 0 0 0 0 0 0 0 0 0;...
    0 1 0 0 0 0 0 0 0 0 0;...
    0 0 1 0 0 0 0 0 0 0 0;...
    0 0 0 1 0 0 0 0 0 0 0;...
    0 0 0 0 1 0 0 0 0 0 0;...
    0 0 0 0 0 1 0 0 0 0 0;...
    0 0 0 0 0 0 1 0 0 0 0;...
    0 0 0 0 0 0 -1 0 0 0 0;...
    0 0 0 0 0 0 1 0 0 0 0;...
    0 0 0 0 0 0 0 1 0 0 0;...
    0 0 0 0 0 0 0 0 1 0 0;...
    0 0 0 0 0 0 0 0 0 1 0;...
    0 0 0 0 0 0 0 0 0 0 1];

q_init = [0;0;0;0;0;0;0;0;0;0;0];
q_rcm_init = convert2rcm(q_init);
Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,q_rcm_init,base_T);
T_eef_init = Arm_Kinematics_init(14).Tran_matrix;
eef_init = T_eef_init(1:3,4);
for i=1:18
    if exist(Arm_Kinematics_init(i).stl_name, 'file') == 2
        VertexData_origin(:,i)=stl2matlab(Arm_Kinematics_init(i).stl_name);
    else
        VertexData_origin(:,i) = {[];[];[]};
    end
end
fclose('all');

base_T1=eye(4);
base_T2=[RotationMatrix_rad(pi/2,[0;0;1]) [-0.8; -0.8; 0]; 0 0 0 1];

% eef_v = [0.5; 0; 0; 0; 0; 0];
v_eef1 = [0.0; 0.3; 0.0];
v_eef2 = [0.3; -0.3; 0.0];
dt = 0.01;
q1 = q_init;
q_rcm1 = convert2rcm(q1);
q2 = q_init;
q_rcm2 = convert2rcm(q2);
figure(1)
hold on
% view(62,28)
view(12,10)
axis equal
iteration = 0;

for j=1:100
    cla
    
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm1,base_T1);
    %     q=[0,0,0,0,0,0,-j/3*pi/2,0,0];
    [J_rcm1,J_car1,J_car_6DoF1,J_all1] = calc_Jacobian_all(Arm_Kinematics1);
    J1 = J_rcm1(1:3,1:4);
    q_dot1 = pinv(J1) * v_eef1;
    q_dot_all1 = [0;0;0;0;0;q_dot1;0;0];
    q1 = q1 + q_dot_all1 *dt;
    q_rcm1 = convert2rcm(q1);
    number = 1;
    for i = 1:18
        T = Arm_Kinematics1(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran1(:,i) = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics1(i).color;
            if ismember(i,[7 8 9 10 11])
                for index = 1 : length(point_clouds{i})
                    point_clouds_tran1{i}(:,index) = R * point_clouds{i}(:,index) + d;
                    center1(:,number) = R * point_clouds{i}(:,index) + d;
                    %                     plot3(center1(1,number),center1(2,number),center1(3,number),'Marker','o')
                    %                     hold on
                    number = number + 1;
                end
            end
            plotSTL(VertexData_tran1(:,i),rgba)
            hold on
            
        end
        
    end
    
    Arm_Kinematics2 = Arm_Kinematics(link_input,joint_input,q_rcm2,base_T2);
    %     q=[0,0,0,0,0,0,-j/3*pi/2,0,0];
    [J_rcm2,J_car2,J_car_6DoF2,J_all2] = calc_Jacobian_all(Arm_Kinematics2);
    J2 = J_rcm2(1:3,1:4);
    q_dot2 = pinv(J2) * v_eef2;
    q_dot_all2 = [0;0;0;0;0;q_dot2;0;0];
    q2 = q2 + q_dot_all2 *dt;
    q_rcm2 = convert2rcm(q2);
    number = 1;
    for i = 1:18
        T = Arm_Kinematics2(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran2(:,i) = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics2(i).color;
            
            if ismember(i,[7 8 9 10 11])
                for index = 1 : length(point_clouds{i})
                    point_clouds_tran2{i}(:,index) = R * point_clouds{i}(:,index) + d;
                    center2(:,number) = R * point_clouds{i}(:,index) + d;
                    
                    %                     plot3(center2(1,number),center2(2,number),center2(3,number),'Marker','o')
                    %                     hold on
                    number = number + 1;
                end
            end
            plotSTL(VertexData_tran2(:,i),rgba)
            hold on
            
        end
    end
    
    collision = false;
    for i = 7:11
        dt1 = delaunayTriangulation(point_clouds_tran1{i}');
        ti1 = pointLocation (dt1,center2');
%                 tetramesh(dt1);
%                 hold on
        if ~isempty(find (isnan(ti1) == 0,1))
            collision = true;
        end
        dt2 = delaunayTriangulation(point_clouds_tran2{i}');
        ti2 = pointLocation (dt2,center1');
%                 tetramesh(dt2);
%                 hold on
        if ~isempty(find (isnan(ti2) == 0,1))
            collision = true;
        end
    end
    
        collision
    if collision
        iteration = iteration +1;
        q1_store(:,iteration) = q_rcm1;
        q2_store(:,iteration) = q_rcm2;
        [p1,p2,d_cp] = find_closest_pnt(center1,center2);
        collision_pnt1(:,iteration) = p1;
        collision_pnt2(:,iteration) = p2;
        p = (p1+p2)/2;
        plot3(p(1),p(2),p(3),'Marker','.','MarkerSize',15);
        hold on
    end
    %     [p1,p2,d_cp] = find_closest_pnt(center1,center2);
    %
    %     if d_cp <= 0.05
    %         iteration = iteration +1;
    %         q1_store(:,iteration) = q_rcm1;
    %         q2_store(:,iteration) = q_rcm2;
    %     end
    %     plot3(p1(1),p1(2),p1(3),'Marker','o');
    %     hold on
    %     plot3(p2(1),p2(2),p2(3),'Marker','o');
    %     hold on
    %     line([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'LineWidth',2);
        axis([ -1.6 0.8 -1.2 0.3 -0.3 0.9])
        drawnow;
end
