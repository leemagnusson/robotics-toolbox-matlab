% dynamics test
% look at dynamics execution time.
% Lee Magnusson 2/19/16

robot = r2   ;
tmax = 1;
tlin = 0:1/60:tmax;
t = linspace(0,tmax,100);
q0 = zeros(1,robot.n);

Kideal = diag([27000,177000,177000,...
               13000,136000,136000,...
               13000,136000,136000,...
               13000,136000,136000,...
               13000,28000,28000,...
               13000,28000,28000,...
               13000,28000,28000,...
               2e7,2e7,2e7,...
               60,60,60]);
zeta = .2*ones(robot.n,1);
B = diag(zeta.*2.*sqrt(diag(robot.inertia(q)).*diag(Kideal)));

torq_fun = @(r,tt,q,qd) -(Kideal*(q-q0)')'-(B*qd')';%robot.gravload(q); %0*ones(size(q));

tic
[to,qo,qod] = robot.fdyn(tlin,torq_fun,q0);
toc


qlin = interp1(to,qo,tlin);
toc
figure(1);
robot.plot(qlin,'workspace',[-1 1 -1 1 -1 1]);
toc
%figure(2);
%plot(tlin,torq_fun(0,0,qlin,0),tlin,-robot.gravload(qlin))

figure(2);
plot(tlin,qlin);