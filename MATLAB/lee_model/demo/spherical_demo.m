%%

tmax = 4;
forces = loadjson('spherical_demo_forces.json');
scale = .01;

t = 0:1/60:tmax;
q = zeros(length(t),r.n);
for i = 2:length(t)
    dt = t(i)-t(i-1);
    F = zeros(1,6);
    for j = 1:length(forces)
        if(ti(i) > forces{j}.tmin && ...
            ti(i) < forces{j}.tmax)
            F = F+forces{j}.F;
        end
    end
    J = [r.jacob0(q(i,:));C];
    dq = jinv_control_fun(r,F*scale,J);
    q(i,:) = q(i-1,:)+dq;
end

%%
figure(1);
r.plot3d(q);