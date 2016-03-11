clc
clear all
close all
% URDF_file= 'necessity_V1.URDF';
URDF_file= 'V1_Arm_URDF.URDF';
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;

Arm_Kinematics_init = Arm_Kinematics(link_input,joint_input,zeros(11,1));
for i=1:13
    [VertexData_origin(:,i),FVCD,isBinary]=stl2matlab(Arm_Kinematics_init(i).stl_name);
end

figure(1)
hold on
view(62,28)
axis equal
for j=1:3
%     cla
    q=[0,0,0,0,0,0,-j/3*pi/2,0,0];
    q_rcm = [q(1) q(2) q(3) q(4) q(5) q(6) q(7) -q(7) q(7) q(8) q(9)];
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm);
    for i = 1:13
        T = Arm_Kinematics1(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
        rgba = Arm_Kinematics1(i).color;
        plotSTL(VertexData_tran,rgba)
        hold on
        fclose('all');
    end
    axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
    drawnow;
end