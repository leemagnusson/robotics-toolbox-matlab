%% Jacobian calculation
% function to export Jacobian
% the jacobian here are all analytical Jacobians
%%
function [J_rcm,J_car,J_car_6DoF,J_all] = calc_Jacobian_all(Arm_Kinematics)
% position of related end effector joints 
eef = Arm_Kinematics(14).Tran_matrix(1:3,4); % end effector at distal wrist joint
rcm = Arm_Kinematics(18).Tran_matrix(1:3,4); % rcm joint
car = Arm_Kinematics(7).Tran_matrix(1:3,4); % cartesian arm end joint at spherical roll joint
wrist = Arm_Kinematics(13).Tran_matrix(1:3,4); % wrist joint at wrist joint

z=zeros(3,length(Arm_Kinematics)-2);
p=zeros(3,length(Arm_Kinematics)-2);
% setup position and z axis of each coordinate frame
for i = 2 : 14
    p(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,4);
    z(:,i) = Arm_Kinematics(i).Tran_matrix(1:3,3);
end
% Cartesian arm jacobian for first five joints
for i = 2 : 6
    J_car(:,i-1) = [cross(z(:,i),car - p(:,i));z(:,i)];
end
% Cartesian arm jacobian for first five joints and roll joint of spherical arm
for i = 2 : 7
    J_car_6DoF(:,i-1) = [cross(z(:,i),car - p(:,i));z(:,i)];
end
% rcm arm 6DoF jacobian
r = wrist - rcm;
r_cross = CPM(-r);
B = r_cross * [z(:,7) z(:,10) z(:,12)];
J_rcm = [B(:,1:2)               z(:,11)         B(:,3)      cross(z(:,13),eef - p(:,13))    cross(z(:,14),eef - p(:,14));...
         z(:,7)     z(:,10)     zeros(3,1)      z(:,12)     z(:,13)                         z(:,14)];

% overall jacobian
J_all = [J_car J_rcm];

end


