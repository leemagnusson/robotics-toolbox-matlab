%% compute twist
% this function takes position and orientation of target and current pose
% and computer 6 by 1 twist from it
% by Haoran Yu 3/28/2016
%%
function [t,p_err,theta_err] = compute_twist(p_t,p_cur,R_t,R_cur)
% iterative inverse kinematics parameters
init_IK_parameters;
% linear velocity
p_err = p_t - p_cur;
if norm(p_err) < p_eps
    v = zeros(3,1);
elseif norm(p_err) < ncycle_t * v_max * dt
    v = p_err/(ncycle_t*dt);
else
    v = v_max * p_err/norm(p_err);
end

% angular velocity
R_err = R_t * R_cur';
theta_err = acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
vect_err = 1/(2*sin(theta_err))*[(R_err(3,2)-R_err(2,3));(R_err(1,3)-R_err(3,1));(R_err(2,1)-R_err(1,2))];
if abs(theta_err) < theta_eps
    omega = zeros(3,1);
elseif abs(theta_err) < ncycle_r * omega_max * dt
    omega = theta_err * vect_err/(ncycle_r*dt);
else
    omega = omega_max * vect_err;
end
t = [v;omega];
end

