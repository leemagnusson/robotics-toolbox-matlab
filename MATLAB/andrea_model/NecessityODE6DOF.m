function dx = NecessityODE6DOF(t,x)
    disp(t);
    global ARM;
    global g0;
    global n_links;
    global n_joints;
%     g0 = RotationMatrix_rad(0.5*pi*t,[0;1;0])*g0;
    ARM1 = ARM;
    ARM1.q(1:n_joints) = x(1:n_joints);
    ARM1.qd(1:n_joints) = x(n_joints+1:2*n_joints);
    u = zeros(n_joints,1);
    g = computeGravity(ARM1,n_links,g0);
    c = computeCentrifugalCorilis(ARM1,n_joints);
    B = computeInertiaMatrix(ARM1,n_joints);
%     ARM1 = NecessityNewtonEuler(ARM1,n,g0,[0;0;0],[0;0;0]);
    ARM1.qdd(1:n_joints) = B\(u-c-g(1:n_joints));
   
    dx = [ARM1.qd(1:n_joints);ARM1.qdd(1:n_joints)];
    clc; disp(dx);
end