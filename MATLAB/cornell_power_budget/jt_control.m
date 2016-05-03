% Jt_control.m
% Lee Magnusson 1/8/16
% Take a look at what it might looks like to do control off a load cell
% on the end effector

function [t,q,Tj] = jt_control

tmax = 1;
t = linspace(0,tmax,tmax*30);
dt = t(2)-t(1);

global q0 r

T0 = r.fkine(q0);
x0 = T0(1:3,4);
xf = x0+[0;-.4;-.5];
Tf = T0;
Tf(1:3,4) = xf;

% traj desired 
tc = ctraj(T0,Tf,length(t));


% x_dot = J*th_dot, T = J'*F

% impedance control to virutal trajectory
[to,qo,qod] = r.fdyn(tmax,@torq_fun,q0);
q = interp1(to,qo,t);
qd = interp1(to,qod,t);
Tj = zeros(size(q));
x = zeros(length(t),6);
for i = 1:length(t)
    Tj(i,:) = torq_fun(r,t(i),q(i,:),qd(i,:));
    x(i,:) = tr2vec(r.fkine(q(i,:)));
end

figure(1); clf;
plot(t,q);

figure(2); clf;
r.plot3d(q);

figure(3); clf;
plot(t,Tj);

figure(4); clf;
plot(t,x);


    function Tj = torq_fun(r,tt,q,qd)
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
        kx = 1000*eye(3);
        bx = 100*eye(6);
        kr = zeros(3);% eye(3);
        diff = t2r(Td)'*t2r(T);
        eul = tr2eul(diff);
        x_dot = J*qd';
        F = [kx*(transl(Td)-transl(T));kr*eul'] + ...
            [bx*(-x_dot)];
        J = r.jacob0(q);
        % null space control which trys to keep the first joint vertical
        T1 = 1*(pi/2-q(1))+1*(-qd(1));
        T3 = .5*(-q(3))+1*(-qd(3));
        
        Tj = J'*F;
        Tj = Tj';
        Tj = Tj + r.gravload(q) + [T1,0,T3,0,0,0,0];
        
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

