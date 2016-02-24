function B = computeInertiaMatrix(ARM,n)
    ARM.qd = 0*ARM.qd;
    B = zeros(n,n);
    g = zeros(3,1);
    for i = 1:1:n
        ARM.qdd = 0*ARM.qdd;
        ARM.qdd(i) = 1;
        ARM = NecessityNewtonEuler(ARM,n,g,[0;0;0],[0;0;0]);
        B(:,i) = ARM.u(1:n)';
    end
end