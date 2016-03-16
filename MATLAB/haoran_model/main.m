%% Test the robot
% Plot the robot at different joint input
% can draw coordinate system
%%
clc
clear all
close all
URDF_file= 'V1_Arm_URDF.URDF';
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;
base_T=eye(4);
Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,zeros(13,1),base_T);
for i=1:18
    if exist(Arm_Kinematics_init(i).stl_name, 'file') == 2
        VertexData_origin(:,i)=stl2matlab(Arm_Kinematics_init(i).stl_name);
    else
        VertexData_origin(:,i) = {[];[];[]};
    end
end
fclose('all');

figure(1)
hold on
view(62,28)
axis equal
for j=1:1
    cla
    q=[0,0,0,0,0,0,0,0,0,pi/3,pi/3]';
    q_rcm = convert2rcm(q);
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T);
    for i = 1:18
        T = Arm_Kinematics1(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics1(i).color;
            plotSTL(VertexData_tran,rgba)
            hold on
            
        end
        if ismember(i,[1 6 7])
            draw_coordinate_system([0.07 0.07 0.07],R,d,'rgb',num2str(i))
            hold on
        end
    end
    T_eef = Arm_Kinematics1(14).Tran_matrix;
    eef = T_eef(1:3,4);
    plot3(eef(1),eef(2),eef(3),'Marker','*','MarkerSize',20)
    hold on
    axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
    drawnow;
end