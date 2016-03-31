% jinv_test.m
% Lee Magnusson 1/8/16
% Take a look at what it might looks like to do control off a load cell
% on the end effector

tmax = 1;
t = linspace(0,tmax,tmax*10);
dt = t(2)-t(1);

global q0 r other_param

r.tool(3,4) = .05;  % virtual center offset

movie = false 
plot3d = false

q00 = q0 + [-1,0,1,0,0,0,0,0,0];%+[-.5,0,0,.1,-.5,0,0];
q00 = q0 + [-2.5,0,1.25,0,-1,0,0,0,0];  % folded config
ws = [-.5,.5,-.4,.65,-.3,.65]
fname = 'virtual_1_in_folded';
q00 = q0 + [.5,0,-.5,0,0,0,0,0,0]; % outstretched
ws = [-.5,.5,-.1,1.5,-.1,.75];
fname = 'virtual_1_in_extend';

T0 = r.fkine(q00);
x0 = T0(1:3,4);
xf = x0+[0;-.1;-.1];
xf = x0;
Tf = T0;
rf = rotx(-2)*roty(0)*t2r(T0); %world frame?
rf = t2r(T0)*rotz(1)*roty(-1)*rotx(1);         % tool frame?

zyx = [0,0;
       1,0;
       1/sqrt(2),1/sqrt(2);
       0,1;
       -1/sqrt(2),1/sqrt(2);
       -1,0;
       -1/sqrt(2),-1/sqrt(2);
       0,-1;
       1/sqrt(2),-1/sqrt(2);
       1,0;
       0,0];
rfs = zeros(3,3,size(zyx,1));
tc = zeros(4,4,size(zyx,1)*length(t));
Tf= T0;
for i = 1:size(zyx,1)
    rfs(:,:,i) = t2r(T0)*roty(zyx(i,1))*rotx(zyx(i,2));
    Tfn = rt2tr(rfs(:,:,i),xf);
    n = length(t);
    tc(:,:,(i-1)*n+(1:length(t))) = ctraj(Tf,Tfn,length(t));
    Tf = Tfn;
end

t = linspace(0,tmax*size(zyx,1),tmax*length(t)*size(zyx,1));

%rf = t2r(T0);
%Tf(1:3,4) = xf;
%Tf = rt2tr(rf,xf);

% traj desired 
%tc = ctraj(T0,Tf,length(t));


% x_dot = J*th_dot th_dot = J^-1*x_dot
% given the starting position and desired x velocity, calculate joint traj
tc(:,:,end+1) = tc(:,:,end);
x_dot = zeros(length(t),6);
q_dot = zeros(length(t),length(q00));
q = zeros(length(t),length(q00));
q(1,:) = q00;
for i = 1:length(t)-1
    % complete feed forward
    x_dot(i,:) = tr2delta(tc(:,:,i),tc(:,:,i+1))/dt;
    
    % update from current position
    x_dot(i,:) = tr2delta(r.fkine(q(i,:)),tc(:,:,i+1))/dt;
    
    J = r.jacob0(q(i,:));
    
    % direct psuedo inverse
    q_dot(i,:) = (pinv(J)*x_dot(i,:)')';
    
    % damped least squares 
    q_dot(i,:) = (inv(J'*J+.0001*diag([1,1,1,1,1,.00001,.00001,.00001,.00001]))*J'*x_dot(i,:)')';
    %q_dot(i,:) = (inv(J'*J+.0001*eye(length(q00)))*J'*x_dot(i,:)')';
    
    q(i+1,:) = q(i,:) + q_dot(i,:)*dt;
end
q_ddot = diff(q_dot,1,1)/dt;
q_ddot(end+1,:) = zeros(1,length(q00));
 
% inverse dynamics for joint torques
Tj = r.rne(q,q_dot,q_ddot);

figure(1); clf;
plot(t,q);
xlabel('time (s)');
ylabel('joint position (rad,m)');
grid on;
legend(other_param.name1,'Location','Best');
print('-dpdf',[fname, '_q_plot']);


figure(2); clf;
if plot3d
    if movie
        r.plot3d(q,'path','robot stl 2015','workspace',ws,'movie','movie');
        system(['ffmpeg -r 30 -i movie/%04d.png -c:v libx264 -preset slow -tune animation -crf 22 ', fname,'.mkv']);

    else 
        r.plot3d(q,'path','robot stl 2015','workspace',ws);
    end
end

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


figure(6); clf; hold on;
plot(t,xc(2:end,:)-transl(r.fkine(q)));
legend('x','y','z');
ylabel('error (m)');
xlabel('time (s)');
grid on
print('-dpdf',[fname, '_x_error_plot']);

rc = t2r(tc);
r0 = t2r(T0);
rr = zeros(size(rc));
zyxpath = zeros(size(rc,3),3);
zyxdesired = zeros(size(rc,3),3);
tpath = r.fkine(q);

for i = 1:size(rr,3)-1
    rr = r0'*rc(:,:,i);
    zyxdesired(i,:) = tr2rpy(rr,'zyx');
    zyxpath(i,:) = tr2rpy(r0'*t2r(tpath(:,:,i)),'zyx');
end

figure(7); clf; hold on;
plot(zyx(:,1),zyx(:,2),'*');
plot(zyxdesired(:,2),zyxdesired(:,3),'^');
plot(zyxpath(:,2),zyxpath(:,3),'.');


axis equal
grid on


figure(8); clf; hold on;
plot(t,q_ddot);

[Pm,Pt] = motor_power_heat(Tj,q_dot,other_param);
Pm_sum = sum(Pm,2);
figure(9); clf; hold on;
plot(t,Pm,t,Pm_sum);

figure(10); clf; hold on;
plot(t,Pt,t,sum(Pt,2));

Pm_mean = mean(sum(Pm,2))
Pt_mean = mean(sum(Pt,2))

accel = max(abs(Tj),[],1) - abs(mean(Tj))

end

