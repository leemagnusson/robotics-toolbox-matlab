% How much inertia/force does a use experience if the joints are gravity 
% comped and frictionless. The user backdrives the arm from the end
% effector

n = 6;
ro = SerialLink(r.links(1:n),'base',r.base);
qo0 = q0(1:n);

%ro.tool = inv(r.links(6).A(0));

%ro.tool = ro.tool*transl(0,0,.1);

Tgrasp = inv(r.links(n).A(0))*transl(0,0,.1);

J = ro.jacobn(qo0);

I = ro.inertia(qo0);
lambda = inv(J*inv(I)*J');


figure(1); clf; hold all;
ro.plot3d(qo0,'wrist','path',stl_dir);
%trplot(ro.fkine(qo0), 'labels', 'xyz', 'arrow','rgb','length',.3)