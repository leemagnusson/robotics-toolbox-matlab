%% Wrist
%
%     Author: Andrea Bajo - andrea.bajo@sri.com
%    Created: 07/01/2015
%   Modified: 
%
%%
clear all;
%% Tool Kinematics
theta = 0;
delta = pi/4;
gamma = 0;
%%
[p,R,J,q,Jq] = NecessityToolDirKin(theta,delta,gamma);
grid on;
axis equal;
axis([-40 40 -40 40 -10 50]);
[EL,AZ] = view;
view([90 0]);
drawnow;
set(gca,'nextplot','replacechildren');
set(gcf,'Renderer','zbuffer');
%%