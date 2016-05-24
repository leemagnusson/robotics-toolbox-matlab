%% compute twist
% this function takes position and orientation of target and current pose
% and computer 6 by 1 twist from it
% [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,R_t,R_cur)
% twist: 6 by 1 twist
% p_err: position error
% theta_err: angle error
% p_t rotation_t: target position and orientation
% p_cur rotation_cur: current position and orientation
%%
function [twist,p_err,theta_err] = ComputeTwist(p_t,p_cur,rotation_t,rotation_cur)
% iterative inverse kinematics parameters
InitIKParameters;
% linear velocity
p_err = p_t - p_cur;
if norm(p_err) < eps_translation
    v = zeros(3,1);
elseif norm(p_err) < num_cycle_translation * v_max * dt
    v = p_err/(num_cycle_translation*dt);
else
    v = v_max * p_err/norm(p_err);
end

% angular velocity
R_err = rotation_t * rotation_cur';
theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
vect_err = 1/(2*sin(theta_err))*[(R_err(3,2)-R_err(2,3));(R_err(1,3)-R_err(3,1));(R_err(2,1)-R_err(1,2))];
if abs(theta_err) < eps_rotation
    omega = zeros(3,1);
elseif abs(theta_err) < num_cycle_rotation * omega_max * dt
    omega = theta_err * vect_err/(num_cycle_rotation*dt);
else
    omega = omega_max * vect_err;
end
twist = [v;omega];
end

