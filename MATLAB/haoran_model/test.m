URDF_file= 'V1_Arm_URDF.URDF';
q=[0,0,0,0,0,0,0,0,0];
Arm_Kinematics1 = Arm_Kinematics(URDF_file,q);
figure(1)
hold on 
view(3)
axis equal
for i = 1:12
fv=stlread(Arm_Kinematics1(i).stl_name);
patch(fv,'FaceColor',       rand(3,1), ...
         'EdgeColor',       'none');
end