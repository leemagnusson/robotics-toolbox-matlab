function g = computeGravity(ARM,n,g)

    ARM.qd = 0*ARM.qd;
    ARM.qdd = 0*ARM.qdd;
    ARM = NecessityNewtonEuler(ARM,n,g,[0;0;0],[0;0;0]);
    g = ARM.u(1:n)';

end