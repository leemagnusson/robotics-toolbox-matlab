% TwoDOF_sim.m
% Lee Magnusson
% 1/26/17

% inverse dynamics for 2 link arm for validation
% eq 4.11 mls94-manipdyn_v1_2

rng(1)  %fix the randonm number generator
t = linspace(0,10,100)';
[q,qd,qdd] = traj_gen(t, 1, 2);

qt = [t, q(:,1), qd(:,1), qdd(:,1)];
qt2 = [t, q(:,2), qd(:,2), qdd(:,2)];

d = TwoDOFDyn;

% these torques should be compared to simscape torques for the same trajectory
tau = d.invDyn(q,qd,qdd);
taut = [t, tau(:,1), tau(:,2)];

figure(1);
plot(t,tau)

% compare forward dynamics solution to original trajectories
sol = ode45(@(tt,y) d.forwardDyn(tt,y,t, tau),t,[qd(1,:),q(1,:)],odeset('reltol',1e-12,'abstol',1e-8));

figure(2);
plot(sol.x,sol.y,t,[qd,q]);

