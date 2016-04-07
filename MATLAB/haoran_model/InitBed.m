%% Input URDF for bed and bed adapter
% input URDF for bed and bed adapter
%%

name_urdf_bed_folder = '2015_drawer_urdf_3a/';
urdf_bed=URDF(strcat(name_urdf_bed_folder,'robots/drawers.urdf'));
urdf_bed_adapter=URDF(strcat(name_urdf_bed_folder,'robots/2015_drawer_urdf_3a.urdf'));
urdf_link_bed = urdf_bed.links;
urdf_joint_bed = urdf_bed.joints;
urdf_link_bed_adapter = urdf_bed_adapter.links;
urdf_joint_bed_adapter = urdf_bed_adapter.joints;

% pull out xyz rpy
for index_bed = 1 : length(urdf_joint_bed)
    xyz_bed(:,index_bed) = urdf_joint_bed{index_bed}.origin.xyz';
    rpy_bed(:,index_bed) = urdf_joint_bed{index_bed}.origin.rpy';
end

frames_bed(:,:,1) = eye(4);
% find bed link up until head link
for index_bed = 2:10
    rotation = RotationAxisAngle([0;0;1],rpy_bed(3,index_bed-1)) * RotationAxisAngle([0;1;0],rpy_bed(2,index_bed-1)) * RotationAxisAngle([1;0;0],rpy_bed(1,index_bed-1));
    translation = xyz_bed(:,index_bed-1);
    frames_bed(:,:,index_bed) = [rotation translation; 0 0 0 1];
    
end
for index_bed = 2 : 5
    frames_bed(:,:,index_bed) = frames_bed(:,:,index_bed-1) * frames_bed(:,:,index_bed);
end

frames_bed(:,:,3) = [frames_bed(1:3,1:3,3) [0;0;0.2984500]; 0 0 0 1];
% 
frames_bed(:,:,4) = [frames_bed(1:3,1:3,4) [0;0;0.4762478]; 0 0 0 1];
% 
frames_bed(:,:,5) = [frames_bed(1:3,1:3,5) [0;0;0.6298416]; 0 0 0 1];

for index_bed = 6 : 10
if index_bed <=8
    frames_bed(:,:,index_bed) = frames_bed(:,:,index_bed-1) * frames_bed(:,:,index_bed);
else
    frames_bed(:,:,index_bed) = frames_bed(:,:,7) * frames_bed(:,:,index_bed);
end
end
% find bed drawer frames 8 in total
num_bed_adapter = 0;
for index_bed = 11 : 18
    num_bed_adapter = num_bed_adapter + 1;
    rotation = RotationAxisAngle([0;0;1],rpy_bed(3,index_bed-1)) * RotationAxisAngle([0;1;0],rpy_bed(2,index_bed-1)) * RotationAxisAngle([1;0;0],rpy_bed(1,index_bed-1));
    translation = xyz_bed(:,index_bed-1);
    frames_bed_adapter_base(:,:,num_bed_adapter) = [rotation translation; 0 0 0 1];
    if index_bed <= 14
        frames_bed_adapter_base(:,:,num_bed_adapter) = frames_bed(:,:,7) * frames_bed_adapter_base(:,:,num_bed_adapter);
    else
        frames_bed_adapter_base(:,:,num_bed_adapter) = frames_bed(:,:,9) * frames_bed_adapter_base(:,:,num_bed_adapter);
    end
end
transformation_bed_base = frames_bed(:,:,2);
num_bed = 0;
% VertexData_bed = [];
for index_bed = 2 : 10
    vertex_bed(:,index_bed-1) = stl2matlab(strcat(name_urdf_bed_folder,'meshes/',urdf_link_bed{index_bed}.name,'.stl'));
    rotation = frames_bed(1:3,1:3,index_bed);
    d = frames_bed(1:3,4,index_bed);
    vertex_bed(:,index_bed-1) = transformSTL(vertex_bed(:,index_bed-1),rotation,d);
end

for index_bed_adapter = 1 : 4
    vertex_bed_adapter(:,index_bed_adapter) = stl2matlab(strcat(name_urdf_bed_folder,'meshes/',urdf_link_bed_adapter{index_bed_adapter}.name,'.stl'));
end

% pull out xyz rpy of bed adapter
for index_bed_adapter = 1 : length(urdf_joint_bed_adapter)
    xyz_bed_adapter(:,index_bed_adapter) = urdf_joint_bed_adapter{index_bed_adapter}.origin.xyz';
    rpy_bed_adapter(:,index_bed_adapter) = urdf_joint_bed_adapter{index_bed_adapter}.origin.rpy';
    axis_bed_adapter(:,index_bed_adapter) = urdf_joint_bed_adapter{index_bed_adapter}.axis.xyz';
end

save('data/vertex_bed.mat','vetex_bed','transformation_bed_base');
save('data/vertex_bed_adapter.mat','vertex_bed_adapter','frames_bed_adapter_base');
save('data/bed_adapter_info.mat','xyz_bed_adapter','rpy_bed_adapter','axis_bed_adapter')

figure(1)
view(3)
axis equal
DrawBed(vertex_bed,[0 0 0 1])
hold on
DrawCoordinateSystem([0.5 0.5 0.5],transformation_bed_base(1:3,1:3),transformation_bed_base(1:3,4),'rgb','b')
hold on

for index_bed = [1 4 8]
    frames_bed_adapter = CalculateBedAdapterFK( [0;0;0],frames_bed_adapter_base(:,:,index_bed));
    DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1])
    hold on
end

for i = 1 : 8
    DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter_base(1:3,1:3,i),frames_bed_adapter_base(1:3,4,i),'rgb','a')
    hold on
end
drawnow;
