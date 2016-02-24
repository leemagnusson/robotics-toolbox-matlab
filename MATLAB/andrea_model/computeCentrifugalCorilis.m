function c = computeCentrifugalCorilis(ARM,n)

    ARM.qdd = 0*ARM.qdd;
    g = zeros(3,1);
    ARM = NecessityNewtonEuler(ARM,n,g,[0;0;0],[0;0;0]);
    c = ARM.u(1:n)';
    
end