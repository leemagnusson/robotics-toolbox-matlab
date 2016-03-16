%% Wrist
%
%     Author: Andrea Bajo - andrea.bajo@sri.com
%    Created: 07/01/2015
%   Modified: 
%
%%
clear all;
STL_names{1} = 'SRI - Wrist - 8mm Yoke 40A.STL';
STL_names{2} = 'SRI - Wrist - 8mm Vertebrae 40A.STL';
STL_names{3} = 'SRI - Wrist - 8mm Jaw 40A (oval).STL';
STL_names{4} = 'SRI - Wrist - 8mm Jaw 40A (oval) 2.STL';
STL_names{5} = 'SRI - Wrist - 8mm Grasper 40A 1.STL';
STL_names{6} = 'SRI - Wrist - 8mm Grasper 40A.STL';
 for i = 1: 1 : size(STL_names,2)
    [p,t,tnorm]=STL_Import(STL_names{i});
    p = p';
    n = length(p(3,:));
    c = ones(1,n)*0.5;
    WRIST.bodies{i}.p = p;
    WRIST.bodies{i}.t = t;
    WRIST.bodies{i}.c = c;
 end
%% Configuration variables
q = zeros(1,6);
i = 1;
%% Wrist parameters
WRIST.parameters.a = 5.3; % mm
WRIST.parameters.b = -6;  % mm 
WRIST.parameters.c = 2.75; % mm 
WRIST.parameters.d = 5.3;
WRIST.parameters.e = 0;
WRIST.parameters.f = 0;
WRIST.parameters.g = 3.798;
save WRIST WRIST;
%% compute initial lengths
WRIST1 = WRIST;
WRIST1.angles = [0,0,0];
WRIST1 = NecessityWristDirKin(WRIST1);
WRIST1 = NecessityWristInvKin(WRIST1);
L0 = WRIST1.L;
for angle = 0:pi/180:pi/2
    WRIST1 = WRIST;
    WRIST1.angles = [angle,angle,angle];
    %% draw writs
    WRIST1 = NecessityWristDirKin(WRIST1);
    %% compute wire displacement
    WRIST1 = NecessityWristInvKin(WRIST1);
    q(i,:) = L0-WRIST1.L;
    i = i + 1;
    grid on;
    axis equal;
    axis([-30 30 -30 30 -10 40]);
    view([-180 0]);
    drawnow;
    title(num2str(angle));
    set(gca,'nextplot','replacechildren');
    set(gcf,'Renderer','zbuffer');
end
% close(writerObj);
q1_lin = interp1([0 90],[q(1,1) q(end,1)],0:1:90);
q2_lin = interp1([0 90],[q(1,2) q(end,2)],0:1:90);
%% Interpolation
X = [0:pi/180:pi/2]';
A = [X.^2 X ones(size(X,1),1)];
Loop1.pos = A\q(:,1);
Loop1.neg = A\q(:,2);
Loop2.pos = A\q(:,3);
Loop2.neg = A\q(:,4);
Loop3.pos = A\q(:,5);
Loop3.neg = A\q(:,6);
save Loop1 Loop1;
save Loop2 Loop2;
save Loop3 Loop3;
%% compute linearity metric
for i = 1:1:size(q1_lin,2)
   q1_err(i) = abs(q(i,1)-q1_lin(i));
   q2_err(i) = abs(q(i,2)-q2_lin(i));
end
linearity_score_100 = [sum(q1_err) sum(q2_err)];
linearity_score_60 = [sum(q1_err(1:60)) sum(q2_err(1:60))];
max_error = [max(q1_err(:)) max(q2_err(:))];
max_indexes = [find(q1_err==max_error(1)) find(q2_err==max_error(2))];
% Loop 1
figure(2);
hold on;
plot(0:1:90,q(:,1),0:1:90,q(:,2),0:1:90,q1_lin,0:1:90,q2_lin);
% plot(0:1:90,q_symb(:,1),'*',0:1:90,q_symb(:,2),'*');
xlabel('\theta_1 [deg]');
ylabel('cable displacement [mm]');
legend('cable 1','cable 2');
grid on;
title('Loop 1');