addpath([pwd, '/../3rdparty/robotics-toolbox-matlab']);
addpath([pwd,'/../3rdparty/toolbox-common-matlab']);
addpath([pwd,'/../3rdparty/jsonlab-1.2/jsonlab']);
% parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 15 8 11;0 18 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
save('parent_number.mat','parent_number');

clc
clear all
close all
% URDF_file= 'necessity_V1.URDF';
URDF_file= 'V1_Arm_URDF.URDF';
urdf_input = URDF(URDF_file);
link_input = urdf_input.links;
joint_input = urdf_input.joints;
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
figure(1)
hold on
view(62,28)
axis equal
steps=[100;30;100;30;100;30;30;1;1;1;1;5;100;100;100;100;100;100];
point_clouds = cell(18,1);
for i = 1:18
    T = Arm_Kinematics_init(i).Tran_matrix;
    R = T(1:3,1:3);
    d = T(1:3,4);
    if isempty(VertexData_origin{1,i}) == 0
        VertexData_tran(:,i) = transformSTL(VertexData_origin(:,i),R,d);
        rgba = Arm_Kinematics_init(i).color;
        f = VertexData_origin{1,i};
        v = VertexData_origin{2,i};
        n = VertexData_origin{3,i};
        number=1;
        for index = 1 : steps(i) : length(f)
            %         for index = 1 : length(point_clouds{i})
            point_clouds{i,1}(:,number) = 1/3 * ([f(1,index),v(1,index),n(1,index)] + [f(2,index),v(2,index),n(2,index)] + [f(3,index),v(3,index),n(3,index)])';
            center = R * point_clouds{i}(:,number) + d;
            %             plot3(center(1),center(2),center(3),'Marker','o')
            %             hold on
            number = number + 1;
        end
        if ismember(i,[8 9 10 11])
            for index1 = 1 : 200
                for index2 = 1 : 200
                    if index1 ~= index2 && index1 <= length(point_clouds{i,1}) && index2 <= length(point_clouds{i,1})
                        if norm(point_clouds{i,1}(:,index1)-point_clouds{i,1}(:,index2)) < 0.02
                            point_clouds{i,1}(:,index2) = [];
                            number =number +1;
                        end
                    end
                end
            end
        end               
    end
end

for i = 1:18
    T = Arm_Kinematics_init(i).Tran_matrix;
    R = T(1:3,1:3);
    d = T(1:3,4);
    if isempty(VertexData_origin{1,i}) == 0
        VertexData_tran(:,i) = transformSTL(VertexData_origin(:,i),R,d);
        rgba = Arm_Kinematics_init(i).color;
        number=1;
        for index = 1 : length(point_clouds{i,1})
            
            center = R * point_clouds{i}(:,index) + d;
            plot3(center(1),center(2),center(3),'Marker','o')
            hold on
            number = number + 1;
        end
        
        plotSTL(VertexData_tran(:,i),rgba)
        hold on
        
        
    end
end

axis([ -0.8 0.8 -1.2 0.3 -0.3 0.9])
drawnow;
% save ('point_clouds_all.mat','point_clouds')