%% quaternion to rotation matrix
% Haoran Yu
% function to convert unit quaternion to rotation matrix
function R=quat2rot_ros(q)
assert(length(q)==4,'Not quaternion')
a=q(4); b=q(1); c=q(2); d=q(3); % intermediate variables for readability
R=[a^2+b^2-c^2-d^2, 2*(b*c-a*d), 2*(b*d+a*c);
    2*(b*c+a*d), a^2-b^2+c^2-d^2, 2*(c*d-a*b);
    2*(b*d-a*c), 2*(c*d+a*b), a^2-b^2-c^2+d^2];
end