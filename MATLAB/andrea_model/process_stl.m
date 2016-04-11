%%
% close all;
cla;
filename = 'necessity_2015_arm_updated.urdf.xacro';
% filename = 'necessity_2015.URDF';
%%
global ARM;
global g0;
global n_links;
global n_joints;
STL_names = importSTLfilesfromURDF(filename);
for i = 1:1:size(STL_names,1)
    strfind_result = strfind(STL_names{i},'/');
    ARM.STL_names{i} = STL_names{i}(strfind_result(1,end)+1:end-1);
    [p,t,tnorm] = STL_Import(ARM.STL_names{i});
    p = p';
    n = length(p(3,:));
    C = ones(1,n)*0.5;
    ARM.bodies{i}.p = p;
    ARM.bodies{i}.t = t;
    ARM.bodies{i}.C = C;
end
%%
ARM.joints_positions = importJointPositionsFromURDF(filename);
ARM.joints_axes = importJointAxisFromURDF(filename);
ARM.joints_orientations = importJointsOrientationsFromURDF(filename);
ARM.links_centers_of_mass = importLinksCentersOfMass(filename);
ARM.links_masses = importLinksMasses(filename);
%% import inertia tensors
values = importLinksInertiafromURDF(filename);
index = 1;
for i = 1:6:size(values,1)
    I = [values(i),values(i+1),values(i+2)
         values(i+1),values(i+3),values(i+4)
         values(i+2),values(i+4),values(i+5)];
    ARM.links_inertia_tensors{index} = I;
    index = index + 1;
end
%% current configuration
ARM.q = zeros(9,1);
ARM.qd = zeros(9,1);
ARM.qdd = zeros(9,1);
ARM.u = zeros(1,size(ARM.joints_axes,1));
%% Newton/Euler Dynamics
g0 = [0;9.81;0];
n_links = 9;
n_joints = 6;
ARM = NecessityNewtonEuler(ARM,n_links,g0,[0;0;0],[0;0;0]);
g = computeGravity(ARM,n_links,g0);
c = computeCentrifugalCorilis(ARM,n_links);
B = computeInertiaMatrix(ARM,n_links);
NecessityDrawRobot(ARM);
%% Dynamics simulation
x0 = [ARM.q(1:n_joints);ARM.qd(1:n_joints);];
option = odeset('RelTol',1e-1,'InitialStep',0.05); 
[t,x] = ode45(@NecessityODE6DOF,[0,.7],x0,option);
writerObj = VideoWriter(strcat('drop_18.avi'));
writerObj.FrameRate = 60;
open(writerObj);
ti = 0:1/writerObj.FrameRate:max(t);
xi = interp1(t,x,ti);
for i = 1:length(ti)
    ARM.q(1:n_joints) = xi(i,1:n_joints)';
    NecessityDrawRobot(ARM);
    title(num2str(ti(i)));
%     axis tight
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
    frame = getframe;
    writeVideo(writerObj,frame);
%     pause;
end
close(writerObj);