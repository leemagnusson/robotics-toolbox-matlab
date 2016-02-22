% Jt_control.m
% Lee Magnusson 1/8/16
% Take a look at what it might looks like to do control off a load cell
% on the end effector

function [t,q,Tj] = jinv_control

tmax = 1;
t = linspace(0,tmax,tmax*30);
dt = t(2)-t(1);

global q0 r

q00 = q0 + [-1,0,1,0,0,0,1];%+[-.5,0,0,.1,-.5,0,0];
T0 = r.fkine(q00);
x0 = T0(1:3,4);
xf = x0+[0;-.1;-.1];
xf = x0;
Tf = T0;
rf = rotx(-2)*roty(0)*t2r(T0); %world frame?
rf = t2r(T0)*rotz(-1)*roty(0);         % tool frame?
%rf = t2r(T0);
%Tf(1:3,4) = xf;
Tf = rt2tr(rf,xf);

% traj desired 
tc = ctraj(T0,Tf,length(t));


% x_dot = J*th_dot th_dot = J^-1*x_dot
% given the starting position and desired x velocity, calculate joint traj
tc(:,:,end+1) = tc(:,:,end);
x_dot = zeros(length(t),6);
q_dot = zeros(length(t),7);
q = zeros(length(t),7);
q(1,:) = q00;
for i = 1:length(t)-1
    x_dot(i,:) = tr2delta(tc(:,:,i),tc(:,:,i+1))/dt;
    J = r.jacob0(q(i,:));
    
    % direct psuedo inverse
    q_dot(i,:) = (pinv(J)*x_dot(i,:)')';
    
    % damped least squares 
    q_dot(i,:) = (inv(J'*J+.0001*diag([1,1,1,1,1,.00001,.00001]))*J'*x_dot(i,:)')';
    %q_dot(i,:) = (inv(J'*J+.0001*eye(7))*J'*x_dot(i,:)')';
    
    q(i+1,:) = q(i,:) + q_dot(i,:)*dt;
end
q_ddot = diff(q_dot,1,1)/dt
q_ddot(end+1,:) = zeros(1,7);

% inverse dynamics for joint torques
Tj = r.rne(q,q_dot,q_ddot);

figure(1); clf;
plot(t,q);

figure(2); clf;
r.plot3d(q);

figure(3); clf;
plot(t,Tj);

xc = transl(tc);
figure(4); clf; hold on;
plot(t,xc(2:end,:));
plot(t,transl(r.fkine(q)),'-.')

figure(5); clf; hold;
ec = tr2eul(tc);
ea = tr2eul(r.fkine(q));
plot(t,ec(2:end,:));
plot(t,ea,'-.');


end

