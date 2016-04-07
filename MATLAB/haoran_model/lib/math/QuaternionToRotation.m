%% quaternion to rotation matrix
% rotation=QuaternionToRotation(quaternion)
% function to convert unit quaternion to rotation matrix
% the quaternion is in the format of ROS standard
% quaternion = [qx qy qz qw]
function rotation=QuaternionToRotation(quaternion)
assert(length(quaternion)==4,'Not quaternion')
qw=quaternion(4); qx=quaternion(1); qy=quaternion(2); qz=quaternion(3); % intermediate variables for readability
rotation=[qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
          2*(qx*qy+qw*qz), qw^2-qx^2+qy^2-qz^2, 2*(qy*qz-qw*qx);
          2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), qw^2-qx^2-qy^2+qz^2];
end