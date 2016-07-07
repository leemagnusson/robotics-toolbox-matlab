%% Input URDF for bed and bed adapter
% This code loads the current version of table (skytron) and table adapter from the
% urdf input in folder '2015_drawer_urdf_3a/'. The hard code numbers are
% not being modified so far because the bed and bed adapter urdf need to be
% modified to match the setup for modular robot design.
% TODO:
% This code should be modified to a class when the table adapter and table
% urdf are updated and finalized.
%%

name_urdf_bed_folder = '../2015_drawer_urdf_3a/';
urdf_bed=URDFParser(strcat(name_urdf_bed_folder,'robots/drawers.urdf'));
urdf_bed_adapter=URDFParser(strcat(name_urdf_bed_folder,'robots/2015_drawer_urdf_3a.urdf'));
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
% find bed link up until head link. Only joint 2 to 10 are chosen as table
% joints. all the rest of them are duplicated joints representing the table
% adapter urdf. The table adapter urdf is chosen from a separate file 'drawers.urdf'
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

% This part of code is for modifying the existing table adapter to the new
% table adapter dimensions
transformation_bed_base = frames_bed(:,:,2);
% frames_bed_adapter_base(1:3,4,1) = [0.2082;0.2111;frames_bed_adapter_base(3,4,1)+ 0.0234];
% frames_bed_adapter_base(1:3,4,2) = [0.2082;-0.2111;frames_bed_adapter_base(3,4,2)+ 0.0234];
% frames_bed_adapter_base(1:3,4,3) = [0.2082-0.25;0.2111;frames_bed_adapter_base(3,4,3)+ 0.0234];
% frames_bed_adapter_base(1:3,4,4) = [0.2082-0.25;-0.2111;frames_bed_adapter_base(3,4,4)+ 0.0234];
% frames_bed_adapter_base(1:3,4,5) = [-0.5961+0.25;0.2111;frames_bed_adapter_base(3,4,5)+ 0.0234];
% frames_bed_adapter_base(1:3,4,6) = [-0.5961+0.25;-0.2111;frames_bed_adapter_base(3,4,6)+ 0.0234];
% frames_bed_adapter_base(1:3,4,7) = [-0.5961;0.2111;frames_bed_adapter_base(3,4,7)+ 0.0234];
% frames_bed_adapter_base(1:3,4,8) = [-0.5961;-0.2111;frames_bed_adapter_base(3,4,8)+ 0.0234];
% frames_bed_adapter_base(:,:,9) = [frames_bed_adapter_base(1:3,1:3,1)*RotationAxisAngle([0;0;1],-pi/2) 1/2 * (frames_bed_adapter_base(1:3,4,1) + frames_bed_adapter_base(1:3,4,2)); 0 0 0 1];
% frames_bed_adapter_base(1,4,9) = 0.2953;

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

% save('data/vertex_bed.mat','vertex_bed','transformation_bed_base');
% save('data/vertex_bed_adapter_new.mat','vertex_bed_adapter','frames_bed_adapter_base');
% save('data/bed_adapter_info.mat','xyz_bed_adapter','rpy_bed_adapter','axis_bed_adapter')

figure(1)
view(3)
axis equal
DrawBed(vertex_bed,[0 1 0 1])
hold on
DrawCoordinateSystem([0.5 0.5 0.5],transformation_bed_base(1:3,1:3),transformation_bed_base(1:3,4),'rgb','b')
hold on

for index_bed = 1:8
    frames_bed_adapter = CalculateBedAdapterFK( [0;-0.0597;0],frames_bed_adapter_base(:,:,index_bed));
    DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,[1 0 0 1])
    hold on
end

for i = 1 : 8
    DrawCoordinateSystem([0.1 0.1 0.1],frames_bed_adapter_base(1:3,1:3,i),frames_bed_adapter_base(1:3,4,i),'rgb',num2str(i))
    hold on
end
grid on
grid minor
view(0, 90)
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
drawnow;
