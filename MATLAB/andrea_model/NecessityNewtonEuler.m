%% Newton/Euler Dynamics
% initial condition
function ARM = NecessityNewtonEuler(ARM,n,g,f_next,m_next)
    p0 = [0;0;0];
    R0 = eye(3);
    w0    = zeros(3,1);
    w0dot = zeros(3,1);
    a0    = zeros(3,1)-g;
    % forward recursion
    for i = 1:1:n
        R_offset = RotationMatrix_rad(ARM.joints_orientations(i,1),[1;0;0])*...
                   RotationMatrix_rad(ARM.joints_orientations(i,2),[0;1;0])*...
                   RotationMatrix_rad(ARM.joints_orientations(i,3),[0;0;1]);
        R01 = R_offset*vrrotvec2mat([ARM.joints_axes(i,:),ARM.q(i)]);
        R = R0*R_offset*vrrotvec2mat([ARM.joints_axes(i,:),ARM.q(i)]);
        p = p0 + R0*ARM.joints_positions(i,:)';
%         p_m = p + R*ARM.links_centers_of_mass(i,:)';
        w  =     R01'*w0 +  ARM.qd(i)*ARM.joints_axes(i,:)';
        wd =  R01'*w0dot + ARM.qdd(i)*ARM.joints_axes(i,:)'+cross(ARM.qd(i)*R01'*w0,ARM.joints_axes(i,:)');
        a  =     R01'*a0 + cross(wd,R01'*ARM.joints_positions(i,:)')+cross(w,cross(w,R01'*ARM.joints_positions(i,:)'));
        a_ci = a + cross(wd,ARM.links_centers_of_mass(i,:)') + cross(w,cross(w,ARM.links_centers_of_mass(i,:)'));
        ARM.p{i} = p;
        ARM.R{i} = R;
        ARM.w{i} = w;
        ARM.wd{i} = wd;
        ARM.a{i} = a;
        ARM.a_ci{i} = a_ci;
        R0 = R;
        p0 = p;
        w0 = w;
        w0dot = wd;
        a0 = a;
    end
    % backward recursion
    ARM.R{n+1} = ARM.R{n};
    for i = n:-1:1
        f   = ARM.R{i}'*ARM.R{i+1}*f_next + ARM.links_masses(i)*ARM.a_ci{i};
        m = ARM.R{i}'*ARM.R{i+1}*m_next ...
              - cross(f,ARM.links_centers_of_mass(i,:)') ...
              + cross(ARM.R{i}'*ARM.R{i+1}*f_next,ARM.links_centers_of_mass(i,:)'-ARM.joints_positions(i+1,:)') ...
              + ARM.links_inertia_tensors{i}*ARM.wd{i} ...
              + cross(ARM.w{i},ARM.links_inertia_tensors{i}*ARM.w{i});
        ARM.f{i} = f;
        ARM.m{i} = m;
        ARM.u(i) = m'*ARM.joints_axes(i,:)'+2.5*ARM.qd(i);
        f_next = f;
        m_next = m;
    end
end