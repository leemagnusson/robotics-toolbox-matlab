
clc 
clear all
load('hernia_joint_angle_speeds.mat')
% arm 0
figure(1)
title('Arm0 before and after filtering')
subplot(2,3,1)
q = plot(time_hernia,q_store{1}(6:11,:));
xlabel('t')
ylabel('joint value')

subplot(2,3,2)

qd = plot(time_hernia(1:end-1),qd_store{1}(6:11,:));
xlabel('t')
ylabel('joint velocity')

subplot(2,3,3)

qdd = plot(time_hernia(1:end-2),qdd_store{1}(6:11,:));
xlabel('t')
ylabel('joint acceleration')
legend({'6', '7', '8', '9', '10', '11'})

subplot(2,3,4)

qs = plot(tsmooth{1},qsmooth{1}(:,6:11));
xlabel('t')
ylabel('joint value')

subplot(2,3,5)
qds = plot(tsmooth{1},qdsmooth{1}(:,6:11));
xlabel('t')
ylabel('joint velocity')

subplot(2,3,6)
qdds = plot(tsmooth{1},qddsmooth{1}(:,6:11));
xlabel('t')
ylabel('joint acceleration')

qd_max = zeros(11,1);
for i = 6 : 11
qd_max(i) = max(abs(qdsmooth{1}(:,i)));
end
% arm 2
figure(2)
title('Arm2 before and after filtering')
subplot(2,3,1)
q = plot(time_hernia,q_store{3}(6:11,:));
subplot(2,3,2)
qd = plot(time_hernia(1:end-1),qd_store{3}(6:11,:));
subplot(2,3,3)
qdd = plot(time_hernia(1:end-2),qdd_store{3}(6:11,:));
legend({'6', '7', '8', '9', '10', '11'})
subplot(2,3,4)
qs = plot(tsmooth{3},qsmooth{3}(:,6:11));
subplot(2,3,5)
qds = plot(tsmooth{3},qdsmooth{3}(:,6:11));
subplot(2,3,6)
qdds = plot(tsmooth{3},qddsmooth{3}(:,6:11));
