function [J_rcm,J_car,J_car_6DoF,J_all] = calc_Jacobian_all(Arm_Kinematics)
eef = Arm_Kinematics(14).Tran_matrix(1:3,4);
rcm = Arm_Kinematics(18).Tran_matrix(1:3,4);
car = Arm_Kinematics(7).Tran_matrix(1:3,4);
wrist = Arm_Kinematics(13).Tran_matrix(1:3,4);

z=zeros(3,length(Arm_Kinematics)-2);
d=zeros(3,length(Arm_Kinematics)-2);
for i = 2 : 14
    d(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,4);
    z(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,3);
end
for i = 2 : 6
    J_car(:,i-1) = [cross(z(:,i),car - d(:,i));z(:,i)];
end

for i = 2 : 7
    J_car_6DoF(:,i-1) = [cross(z(:,i),car - d(:,i));z(:,i)];
end

r = wrist - rcm;
r_cross = CPM(-r);
B = r_cross * [z(:,7) z(:,10) z(:,12)];
J_rcm = [B(:,1:2)               z(:,11)         B(:,3)      cross(z(:,13),eef - d(:,13))    cross(z(:,14),eef - d(:,14));...
         z(:,7)     z(:,10)     zeros(3,1)      z(:,12)     z(:,13)                         z(:,14)];
     
J_all = [J_car J_rcm];

end


