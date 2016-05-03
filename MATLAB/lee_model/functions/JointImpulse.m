function t_p = JointImpulse(r,q,v_i)
% look at impulse torque required
% lambda (task space inertia)
% f_p (task space impulse)
% v_i (initial velocity)
% f_p = lambda*v_i
% lambda = (J*I^-1*J')^-1
% t_p (joint space impulse torque)
% t_p = J'*f_p


J = r.jacob0(q);
I = r.inertia(q);
lambda = inv(J*inv(I)*J');
% assume v_i in 3 directions no rotational velocity
f_p = lambda*[v_i;v_i;v_i;0;0;0];
t_p = J'*f_p;