% Structured Trajectory for Accuracy testing and requirements definition
% Created: 2017-01-24 David Monteverde
% Updates:
% 2017-01-26 DRM - added parameter merging_steps to account for
%      settling of inverse kinematics (before this number was 1 by default)
% 2017-02-07 DRM - remove modified helix pattern('flowers'); options for helix are
%    now 'fast' and 'slow'

clc
close all; clear all

%% choose desired pattern
% uncomment one choice:
pattern = 'helixfast';
%pattern = 'helixslow';

%% choose desired pose
% uncomment one choice:
pose = 'toolreach_mid';
%pose = 'toolreach_min';
%pose = 'toolreach_max';

%% parameters for base helical (circular) pattern:
d = 0.030; % m, circle (helix) diameter
samprate = 400; % Hz, sampling rate
% circle (projection of helix) defined on x-y plane, then trajectory is rotated with following Euler angles:
alpha = deg2rad(0); % rotation about x axis
beta = deg2rad(30);  % rotation about y axis
% cruising speeds:
switch pattern
    case 'helixfast'
        v = 0.060;  % m/s, TSE linear speed
    case 'helixslow'
        v = 0.015;  % m/s, TSE linear speed
end

dReach_lim = 0.054; % m, minimum allowed distance between TSE and RCM, 
%    given by: allowable speed limit, tool mass and nominal trajectory speed. 
%    For now just used in adjusting q_toolreach_min, and later verifying in last plot. 
%    dReach_lim = 0.054 assumes tool mass is 350 g and TSE speed is 60 mm/s

%% Calcs:
% construct base helix trajectory as follows:
% begin and end at zero speed [t0]
% half turn at constant acceleration [t0 t1]
% two turns at constant (cruising) speed [t1 t2]
% half turn at constant deceleration [t2 t3]
% so helix has a total of 3 turns
% depth of helix (along z) is same as diameter

r = d/2; % radius
N = 3; % number of turns
pitch = d/N; % m, helix pitch

phi = atan(pitch/(2*pi*r)); % helix pitch angle

vRadial = v*cos(phi); % velocity projected on x-y plane 
vAxial = v*sin(phi); % velocity projected on z direction

wRadial = vRadial/r; % circle angular velocity (rad/s, on x-y plane)

theta_ramping = pi; % half circle
theta_coasting = 2*pi*(N-1); % two turns
% angular acceleration (for circle on x-y plane)
aRadial = (wRadial^2)/(2*theta_ramping);

dist_ramping = pitch/2; % corresponding to half circle
aAxial = (vAxial^2)/(2*dist_ramping);  % linear acceleration (along z)

% time period durations
t1 = wRadial/aRadial;  % during acceleration
t2 = theta_coasting/wRadial; % during coasting
t3 = t1; % during deceleration

% time vectors
tstep = 1/samprate; % time step (in seconds)
t01 = (0:tstep:t1)'; % acceleration
t12 = (tstep:tstep:t2)'; % coasting
t23 = (tstep:tstep:t3)'; % deceleration

% corresponding angles (as functions of time, circle motion on x-y plane)
theta01 = 0.5*aRadial*t01.^2; % acceleration
theta12 = wRadial*t12; % coasting
theta23 = wRadial*t23 - 0.5*aRadial*t23.^2; % deceleration

% corresponding distances on z-direction
z01 = 0.5*aAxial*t01.^2; % acceleration
z12 = vAxial*t12; % coasting
z23 = vAxial*t23 - 0.5*aAxial*t23.^2; % deceleration

% join sections
t = [t01
    t01(end)+t12
    t01(end)+t12(end)+t23];
theta = [theta01
    theta01(end)+theta12
    theta01(end)+theta12(end)+theta23];
z = [z01
    z01(end)+z12
    z01(end)+z12(end)+z23];

n = length(t); % number of steps
% generate pattern
x = zeros(n,1);
y = zeros(n,1);
z = z - median(z); % re-center vector about zero

% generate circle (planar projection of helix, on x-y plane)
for i=1:n
    x(i) = r*cos(theta(i));
    y(i) = r*sin(theta(i));
end

coasting_start = length(t01); % index
coasting_end = length(t01)+length(t12)+1; % index

% plot position vs time
figure(1)
subplot(3,1,1)
plot(t,x)
hold on
plot(t(coasting_start),x(coasting_start),'ko')
plot(t(coasting_end),x(coasting_end),'ko')
grid minor
xlabel('time (s)'); ylabel('x (m)')

subplot(3,1,2)
plot(t,y)
hold on
plot(t(coasting_start),y(coasting_start),'ko')
plot(t(coasting_end),y(coasting_end),'ko')
grid minor
xlabel('time (s)'); ylabel('y (m)')

subplot(3,1,3)
plot(t,z)
hold on
plot(t(coasting_start),z(coasting_start),'ko')
plot(t(coasting_end),z(coasting_end),'ko')
grid minor
xlabel('time (s)'); ylabel('z (m)')

% plot speed vs time
figure(2)
speed = zeros(n-1,1);  % initialize
for i=1:n-1
    speed(i) = norm([x(i+1)-x(i)  y(i+1)-y(i)  z(i+1)-z(i)]) / tstep;
end  
plot(t(2:end),speed)
hold on
plot(t(coasting_start),speed(coasting_start),'ko')
plot(t(coasting_end),speed(coasting_end),'ko')
grid minor
xlabel('time (s)')
ylabel('speed (m/s)')

% plot trajectory
figure(3)
plot3(x,y,z)
hold on
plot3(x(coasting_start),y(coasting_start),z(coasting_start),'ko')
plot3(x(coasting_end),y(coasting_end),z(coasting_end),'ko')
axis equal; grid minor
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')
axis tight
view(30,45)

% rotation matrices about principal axes
Rx = [ 1      0            0;
       0  cos(alpha)  -sin(alpha);
       0  sin(alpha)   cos(alpha)];
   
Ry = [ cos(beta)  0  sin(beta);
           0      1     0;
      -sin(beta)  0  cos(beta)];
  
% rotate plane on which path was defined
pattern_path = zeros(3,n);
for i=1:n
    pattern_path(:,i) = Rx*Ry*[x(i) y(i) z(i)]';
end

%% load robot
load('vertex_arm_origin_1.5.mat')
load('arm_version_1.5.mat')
% modify URDF
%**** in RobotClass (>>open RobotClass), move xyz_ from private to public
% when using mockup weighted tool, origin of optical marker corresponds to
% proximal wrist
% per Bharat, need to adjust following parameter in URDF (distance from proximal wrist to back plane of carriage):

% **********USE THIS IF NEEDED **********
%robot_object.xyz_(3,12) = 0.5174; % FARO measurement of mockup tool 2017-01-20


%% define robot poses 
% "extended" pose, with tool translate in 3 positions:
% q_toolreach_mid is halfway between J8 joint limits
% q_toolreach_min is near J8 minimum-reach joint limit
% q_toolreach_max is near J8 maximum-reach joint limit
% 
d2r = deg2rad(1);
r2d = rad2deg(1);

q_toolreach_mid=[ 
    -160*d2r    % J1
    0           % J2
    110*d2r  %robot_object.joint_limit_(2,3+1)   % J3
    0           % J4
    0*d2r     % J5
    0*d2r      % J6
    mean(robot_object.joint_limit_(:,10)) - 40*d2r  % J7
    mean(robot_object.joint_limit_(:,11))   % tool translate
    0   % 
    0   % 
    0;0;0]; %

q_toolreach_min = q_toolreach_mid;
q_toolreach_min(8) = robot_object.joint_limit_(1,11) + 0.076; % adjust using dReach and dReach_lim

q_toolreach_max = q_toolreach_mid;
q_toolreach_max(8) = robot_object.joint_limit_(2,11) - 0.076;

%% select which pose to use:
switch pose
    case 'toolreach_mid'
        q = q_toolreach_mid;
    case 'toolreach_min'
        q = q_toolreach_min;
    case 'toolreach_max'
        q = q_toolreach_max;
end
        
%% perform inverse kinematics based on helical path
%filtering = 'yes'; % for applying filter post-IK

%robot_object.transformation_base_ = eye(4);
robot_object.CalculateFK(q);

translation_eef_init = robot_object.frames_(1:3,4,14); % eef position
rotation_eef_init = robot_object.frames_(1:3,1:3,14); % eef orientation

for i = 1:n
    position_path(:,i) = translation_eef_init - pattern_path(:,i);  % shift position (center of helix is at nominal position)
    rotation_path(:,:,i) = rotation_eef_init;  % keep eef orientation constant
end

% draw robot in reference configuration, and helical path
figure(11)
hold on
%camzoom(3)
% end effector (distal wrist) is 14
robot_object.DrawRobot(vertex_arm_origin,[0+1 3+1 5+1 7+1], 1.0);
grid minor; axis equal
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
light('Position',[1 3 2]);
light('Position',[-3 -1 -3]);
 
% scatter3
plot3(position_path(1,:),position_path(2,:),position_path(3,:))
% draw axes at first pose of path
%DrawCoordinateSystem([0.04 0.04 0.04],rotation_path(:,:,1),position_path(:,1),'rgb','init')

% perform inverse kinematics calculations (first step brings robot eef to path)
% to modify convergence criteria, change parameters in:
% C:\Users\dmonteverde\Google Drive\MATLAB\arm_analysis\MATLAB\haoran_model\init\InitIKParameters.m
qi = q; % initialize
for i = 2:n
    i
    qi(1:11,i) = robot_object.InverseKinematics(qi(1:11,i-1),position_path(:,i),rotation_path(:,:,i),'Spherical 6');
end
qi = qi';

% clean up results: remove first couple of steps, which is jump from center
% of pattern to actual path
merging_steps = 3;
t(1:merging_steps) = [];
qi(1:merging_steps,:) = [];

pattern_jointspace = [t qi];
% make sure first 2 rows are equal (makes running on robot easier)
pattern_jointspace = [pattern_jointspace(1,:);
                      pattern_jointspace];

% plot joint values vs time
figure(12)
subplot(3,1,1)
plot(t,qi(:,6))
xlabel('t (s)'); ylabel('J6 (rad)')
grid minor
subplot(3,1,2)
plot(t,qi(:,7))
xlabel('t (s)'); ylabel('J7 (rad)')
grid minor
hold on
plot(t,robot_object.joint_limit_(1,8)*ones(n-merging_steps,1),'--')
plot(t,robot_object.joint_limit_(2,8)*ones(n-merging_steps,1),'--')
subplot(3,1,3)
plot(t,qi(:,8))
xlabel('t (s)'); ylabel('J8 (m)')
grid minor
hold on
plot(t,robot_object.joint_limit_(1,11)*ones(n-merging_steps,1),'--')
plot(t,robot_object.joint_limit_(2,11)*ones(n-merging_steps,1),'--')

%% calculate wrist trajectory

clear traj_wrist
for i=1:length(qi)
    q = qi(i,:)';
    robot_object.CalculateFK(q);
    traj_wrist(i,:) = robot_object.frames_(1:3,4,13)'; % trajectory of proximal wrist (index=13)
end

pattern_taskspaceWrist = [t traj_wrist];
   
% plot wrist-to-RCM distance vs time
locRCM = robot_object.frames_(1:3,4,18); % location of RCM
clear dReach
for i=1:length(traj_wrist)
    dReach(i) = norm(traj_wrist(i,:) - locRCM');
end
dReach = dReach'; % make column vector

figure(13)
plot(t,dReach)
hold on; grid minor
plot(t,dReach_lim*ones(length(t),1),'--')
xlabel('t (s)')
ylabel('RCM-to-TSE distance (m)')


%% save results
switch pattern
    case 'helixfast'
        switch pose
            case 'toolreach_mid'
                save('helixfast_midp_jointspace.mat','pattern_jointspace')
                save('helixfast_midp_taskspaceWrist.mat','pattern_taskspaceWrist')
            case 'toolreach_min'
                save('helixfast_highp_jointspace.mat','pattern_jointspace')
                save('helixfast_highp_taskspaceWrist.mat','pattern_taskspaceWrist')
            case 'toolreach_max'
                save('helixfast_lowp_jointspace.mat','pattern_jointspace')
                save('helixfast_lowp_taskspaceWrist.mat','pattern_taskspaceWrist')
        end
    case 'helixslow'
        switch pose
            case 'toolreach_mid'
                save('helixslow_midp_jointspace.mat','pattern_jointspace')
                save('helixslow_midp_taskspaceWrist.mat','pattern_taskspaceWrist')
            case 'toolreach_min'
                save('helixslow_highp_jointspace.mat','pattern_jointspace')
                save('helixslow_highp_taskspaceWrist.mat','pattern_taskspaceWrist')
            case 'toolreach_max'
                save('helixslow_lowp_jointspace.mat','pattern_jointspace')
                save('helixslow_lowp_taskspaceWrist.mat','pattern_taskspaceWrist')
        end
end


