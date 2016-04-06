%% quaternion to rotation matrix
% function to convert unit quaternion to rotation matrix
% the quaternion is in the format of ROS
% x y z qx qy qz qw
function R=quat2rot_ros(q)
assert(length(q)==4,'Not quaternion')
qw=q(4); qx=q(1); qy=q(2); qz=q(3); % intermediate variables for readability
R=[qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
    2*(qx*qy+qw*qz), qw^2-qx^2+qy^2-qz^2, 2*(qy*qz-qw*qx);
    2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), qw^2-qx^2-qy^2+qz^2];
end