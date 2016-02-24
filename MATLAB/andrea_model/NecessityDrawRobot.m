function NecessityDrawRobot(ARM)
%     disp(ARM.q);
    global n;
    global g0;
    global n_links;
    figure(1); cla; hold on; grid on;
    % base
    p0 = [0;0;0];
    R0 = eye(3);
    h1 = trisurf(ARM.bodies{1}.t,ARM.bodies{1}.p(1,:),ARM.bodies{1}.p(2,:),ARM.bodies{1}.p(3,:),ARM.bodies{1}.C);
    draw_coordinate_system([0.2 0.2 0.2],R0,p0,['r' 'g' 'b'],'w');
    alpha(h1,1);
    for i=1:1:n_links
        R_offset = RotationMatrix_rad(ARM.joints_orientations(i,1),[1;0;0])*...
                   RotationMatrix_rad(ARM.joints_orientations(i,2),[0;1;0])*...
                   RotationMatrix_rad(ARM.joints_orientations(i,3),[0;0;1]);
        R = R0*R_offset*angvec2r(ARM.q(i),ARM.joints_axes(i,:)/norm(ARM.joints_axes(i,:)));
        p = p0 + R0*ARM.joints_positions(i,:)';
        ARM.p{i} = p;
        ARM.R{i} = R;
        p_pad = [p(1)*ones(1,size(ARM.bodies{i+1}.p,2))
                 p(2)*ones(1,size(ARM.bodies{i+1}.p,2))
                 p(3)*ones(1,size(ARM.bodies{i+1}.p,2))];
        ARM.bodies{i+1}.p = p_pad+R*ARM.bodies{i+1}.p;
        % draw gravity vectors
        p_m = p + R*ARM.links_centers_of_mass(i,:)';
        ARM.pm{i} = p_m;
        ARM.g{i} = 9.81*ARM.links_masses(i)*[0;0;-1];
        p_m1 = p_m + 0.25*ARM.links_masses(i)*g0/norm(g0);
        line([p_m(1) p_m1(1)],[p_m(2) p_m1(2)],[p_m(3) p_m1(3)],'LineWidth',2);
        text(p_m1(1),p_m1(2),p_m1(3),num2str(i));
        % draw axes
        j1 = p + 0.2*R*ARM.joints_axes(i,:)';
        j2 = p - 0.2*R*ARM.joints_axes(i,:)';
        line([j1(1) j2(1)],[j1(2) j2(2)],[j1(3) j2(3)],'Color','b');
        % compute joint torques
        R0 = R;
        p0 = p;
        h = trisurf(ARM.bodies{i+1}.t,ARM.bodies{i+1}.p(1,:),ARM.bodies{i+1}.p(2,:),ARM.bodies{i+1}.p(3,:),ARM.bodies{i+1}.C);
         draw_coordinate_system([0.2 0.2 0.2],R,p,['r' 'g' 'b'],num2str(i));
        alpha(h,0.5);
        view([-90 0]);
        axis equal;
        axis([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
        drawnow;
    %     pause;
    end
end