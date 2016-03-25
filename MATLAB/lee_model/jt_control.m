% Jt_control.m
% Lee Magnusson 1/8/16
% Take a look at what it might looks like to do control off a load cell
% on the end effector

function [t,q,Tj,r6] = jt_control

tmax = 2;
t = linspace(0,tmax,tmax*60);
dt = t(2)-t(1);

global q0 r robot_dir

% 6dof
r6 = SerialLink(r.links(1:6),'base',r.base);
r6.gravity = r.gravity;
r_base = add_base_axes(r6);
r_base.gravity = 0*r_base.gravity;

T0 = r6.fkine(q0(1:6));
x0 = T0(1:3,4);
xf = x0+[0;-.5;-.2];
Tf = T0;
Tf(1:3,4) = xf;

% traj desired 
tc = ctraj(T0,Tf,length(t));




% x_dot = J*th_dot, T = J'*F

% impedance control to virutal trajectory
[to,qo,qod] = r6.fdyn(tmax,@torq_fun,q0(1:6));
q = interp1(to,qo,t);
qd = interp1(to,qod,t);
qdd = zeros(size(q));
Tj = zeros(size(q));
F = zeros(length(t),6);
Fbase = zeros(length(t),6);
x = zeros(length(t),6);
for i = 1:length(t)
    [Tj(i,:),F(i,:),Fbase(i,:),qdd(i,:)] = torq_fun(r6,t(i),q(i,:),qd(i,:));
    x(i,:) = tr2vec(r6.fkine(q(i,:)));
end

figure(1); clf;
plot(t,q);

figure(3); clf;
plot(t,Tj);

figure(4); clf;
plot(t,x);

figure(5); clf;
plot(t,F);

figure(6); clf;
plot(t,Fbase);

figure(2); clf;
% r6.plot3d(q,'path',[robot_dir '/stl'],'workspace',[-1 1 -1 1 -1 1]);
% 
% robot_movie(r6,q,[-1 1 -1 1 -1 1]);


    function [Tj,F,Fbase,qdd] = torq_fun(r,tt,q,qd)
        T = r.fkine(q);
        tt
        ind = find(tt<t,1) -1;
        if isempty(ind)
            ind = length(t)-1;
            xx = 1;
        else
            xx = 1-(tt-t(ind))/dt;
        end
        Td = tc(:,:,ind)*xx + tc(:,:,ind+1)*(1-xx);
        J = r.jacob0(q);
        %k = diag(100*[1,1,1,0,0,0]);
        %F = k*(tr2vec(Tf)-tr2vec(T));
        kx = 100*eye(3);
        bx = 10*eye(6);
        kr = zeros(3);% eye(3);
        diff = t2r(Td)'*t2r(T);
        eul = tr2eul(diff);
        x_dot = J*qd';
        F = [kx*(transl(Td)-transl(T));kr*eul'] + ...
            [bx*(-x_dot)];
        J = r.jacob0(q);
    
        J6 = J(:,1:6);
        gl = r.gravload(q)';
        
        Tj = gl;
        
        
        Tj(1:6) = J6'*F + gl(1:6);
        Tj = Tj';
        
        Ibase =  r_base.inertia([zeros(1,6),q]);
        
        % iterate here ?
        qdd = r.accel(q,qd,Tj);
        tau = rne(r_base, [zeros(1,6),q], [zeros(1,6),qd], zeros(1,length(q)+6));
        Fbase = Ibase(1:6,end-5:end)*qdd + tau(1:6)';
        Fbase_reorder = Fbase([1,3,5,2,4,6]);
        Fnew = F+Fbase_reorder;
        
        Tj(1:6) = J6'*Fnew + gl(1:6);
        
        
  %      Tj = Tj';
%         % null space control which trys to keep the first joint vertical
%         T1 = 1*(pi/2-q(1))+1*(-qd(1));
%         T3 = .5*(-q(3))+1*(-qd(3));
%         
%         Tj = J'*F;
%         Tj = Tj';
%         Tj = Tj + r.gravload(q) + [T1,0,T3,0,0,0,0,0,0];
        
%         gl = r.gravload(q);
%         gl(1:end-2) = 0;
%         k = 1000;
%         qk = q;
%         qk(1:end-2) = 0;
%         Tj = zeros(size(q)) + gl -k*qk;
        
        % admittance control
%         x_dot_desired = [0;-1;0;0;0;0];
%         qd_desired = [];
%         if (rank(J) < 6 || cond(J) < .1)
%             qd_desired = J'*x_dot_desired;
%         else
%             qd_desired = J\x_dot_desired;
%         end
%         b = 100*eye(7);
%         Tj = r.gravload(q) + (b*(qd_desired - qd'))';
    end

    function v = tr2vec(T)
        v = [transl(T);tr2eul(T)'];
    end
end

