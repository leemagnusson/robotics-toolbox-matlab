%% rot2quat: Rotation matrix to quaternion
% This function converts a rotation matrix R into a normalized quaternion
% q. Taken from Siciliano's textbook
% Haoran Yu

function q=rot2quat(R)
%     eta=sqrt(R(1,1)+R(2,2)+R(3,3)+1)/2;
%     if sign(R(3,2)-R(2,3))>=0
%         s1=1;
%     else
%         s1=-1;
%     end
%     if sign(R(1,3)-R(3,1))>=0
%         s2=1;
%     else
%         s2=-1;
%     end
%     if sign(R(2,1)-R(1,2))>=0
%         s3=1;
%     else
%         s3=-1;
%     end
%     n=0.5*[s1*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
%            s2*sqrt(R(2,2)-R(1,1)-R(3,3)+1);
%            s3*sqrt(R(3,3)-R(2,2)-R(1,1)+1)];
%     q=[eta;n];
tr = R(1,1) + R(2,2) + R(3,3);
if tr > 0
    S = sqrt(tr+1.0) * 2;
    qw = 0.25 * S;
    qx = (R(3,2) - R(2,3)) / S;
    qy = (R(1,3) - R(3,1)) / S;
    qz = (R(2,1) - R(1,2)) / S;
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
    qw = (R(3,2) - R(2,3)) / S;
    qx = 0.25 * S;
    qy = (R(1,2) + R(2,1)) / S;
    qz = (R(1,3) + R(3,1)) / S;
elseif R(2,2) > R(3,3)
    S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
    qw = (R(1,3) - R(3,1)) / S;
    qx = (R(1,2) + R(2,1)) / S;
    qy = 0.25 * S;
    qz = (R(2,3) + R(3,2)) / S;
else
    S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
    qw = (R(2,1) - R(1,2)) / S;
    qx = (R(1,3) + R(3,1)) / S;
    qy = (R(2,3) + R(3,2)) / S;
    qz = 0.25 * S; 
end
q = [qw;qx;qy;qz];
end