%% RotationToQuaternion: Rotation matrix to quaternion
% quaternion=RotationToQuaternion(rotation)
% This function converts a rotation matrix R into a normalized quaternion
% rotation: The rotation matrix is 3 X 3
% quaternion: The quaternion = [qx, qy, qz, qw]

function quaternion=RotationToQuaternion(rotation)
trace = rotation(1,1) + rotation(2,2) + rotation(3,3);
if trace > 0
    S = sqrt(trace+1.0) * 2;
    qw = 0.25 * S;
    qx = (rotation(3,2) - rotation(2,3)) / S;
    qy = (rotation(1,3) - rotation(3,1)) / S;
    qz = (rotation(2,1) - rotation(1,2)) / S;
elseif rotation(1,1) > rotation(2,2) && rotation(1,1) > rotation(3,3)
    S = sqrt(1.0 + rotation(1,1) - rotation(2,2) - rotation(3,3)) * 2;
    qw = (rotation(3,2) - rotation(2,3)) / S;
    qx = 0.25 * S;
    qy = (rotation(1,2) + rotation(2,1)) / S;
    qz = (rotation(1,3) + rotation(3,1)) / S;
elseif rotation(2,2) > rotation(3,3)
    S = sqrt(1.0 + rotation(2,2) - rotation(1,1) - rotation(3,3)) * 2;
    qw = (rotation(1,3) - rotation(3,1)) / S;
    qx = (rotation(1,2) + rotation(2,1)) / S;
    qy = 0.25 * S;
    qz = (rotation(2,3) + rotation(3,2)) / S;
else
    S = sqrt(1.0 + rotation(3,3) - rotation(1,1) - rotation(2,2)) * 2;
    qw = (rotation(2,1) - rotation(1,2)) / S;
    qx = (rotation(1,3) + rotation(3,1)) / S;
    qy = (rotation(2,3) + rotation(3,2)) / S;
    qz = 0.25 * S; 
end
quaternion = [qx;qy;qz;qw];
end