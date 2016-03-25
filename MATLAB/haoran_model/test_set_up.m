clc
clear all
close all
load('URDF_info.mat')
load('VertexData_origin.mat')
load('VertexData_Hernia_Body.mat');
% load('q_init_setup_hernia_SW.mat');
% load('q_init_setup_hernia_optimized3.mat');
q_setup = [0 0 0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0 0 0;]';
ARM_setup_hernia
figure(1)
hold on
axis equal
view(3)
view(-2,45)
draw_coordinate_system([0.1 0.1 0.1],eye(3),[0;0;0],'rgb','w')
hold on
draw_coordinate_system([0.1 0.1 0.1],R_Base1,Base1,'rgb','b1')
hold on
draw_coordinate_system([0.1 0.1 0.1],R_Base2,Base2,'rgb','b2')
hold on
draw_coordinate_system([0.1 0.1 0.1],R_Base3,Base3,'rgb','b3')
hold on
draw_coordinate_system([0.02 0.02 0.02],R_Trocar1,Trocar1,'rgb','t1')
hold on
draw_coordinate_system([0.02 0.02 0.02],R_Trocar2,Trocar2,'rgb','t2')
hold on
draw_coordinate_system([0.02 0.02 0.02],R_Trocar3,Trocar3,'rgb','t3')
hold on
VertexData_hernia_tran = transformSTL(VertexData_Hernia_Body,R_Hernia_Body,Hernia_Body);
rgba = [0 0 1 0.1];
plotSTL(VertexData_hernia_tran,rgba)
hold on
for index = 1:3
    q_rcm = convert2rcm(q_setup(:,index));
    Arm_Kinematics1 = Arm_Kinematics(link_input,joint_input,q_rcm,base_T_setup(:,:,index));
    index_jnt_limit = 1;
    for i = 1:length(Arm_Kinematics1)
        T = Arm_Kinematics1(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics1(i).color;
            plotSTL(VertexData_tran,rgba)
            hold on
            
        end
        if ~isempty(Arm_Kinematics1(i).jnt_limit)
            jnt_limit(:,index_jnt_limit) = Arm_Kinematics1(i).jnt_limit;
            index_jnt_limit = index_jnt_limit + 1;
        end
    end
    index_q = 1;
    for i = 1 : length(q_rcm)
    jnt_limit_percentage(index_q,index) = (q_rcm(index_q) - jnt_limit(1,i))/(jnt_limit(2,i)-jnt_limit(1,i));
    index_q = index_q + 1;
    end
%     T_eef = Arm_Kinematics1(14).Tran_matrix;
%     eef = T_eef(1:3,4);
%     draw_coordinate_system([0.03 0.03 0.03],Arm_Kinematics1(7).Tran_matrix(1:3,1:3),Arm_Kinematics1(18).Tran_matrix(1:3,4),'rgb',num2str(i))
%     hold on
%     plot3(eef(1),eef(2),eef(3),'Marker','*','MarkerSize',20)
%     hold on
%     axis([ -0.6 0.8 -0.8 0.9 -0.4 0.9])
    
end
plot3(Hernia(1),Hernia(2),Hernia(3),'Marker','o','MarkerSize',10)
axis([ -0.6 0.8 -0.8 0.9 -0.4 0.9])
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
drawnow;
jnt_limit_percentage