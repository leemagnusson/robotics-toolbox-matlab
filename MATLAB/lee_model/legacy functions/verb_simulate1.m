%verb_simulate_1.m
% Lee Magnusson 12/29/15
% Simulate a movement of the virtual center

%%
t = linspace(0,.5,100);
qs = q0;
qf = q0;
qf(end) = -pi/4;
qf(6) = -pi/4;
ws = [-1,1,-.5,1.25,-.1,1];
name = 'case1';

% qs = q0;
% qf = q0;
% qs(1) = -pi*.4;
% qs(3) = pi*.4;
% qs(end) = -pi*.5;
% ws = [-1,1,-.5,1.25,-.4,1];
% name = 'case2'


[q,qd,qdd] = jtraj(qs,qf,t);

figure(10); clf;
r.plot3d(qs,'workspace',ws);
group = findobj(gca,'Tag',r.name);
h = group.UserData;
h2link = copyobj(h.link(2:end),gca);
set([h2link.Children],'FaceAlpha',.5)
r.animate(qf);%,'workspace',[-1,1,-.5,1.25,-.1,1]
set(gcf,'Renderer','painters')
print('-deps2c',['images/movement',name]) 
%%
figure(11); clf;
plot(q(:,end));


%% inverse dynamics

tau = r.rne(q,qd,qdd);
figure(12);
plot(t,tau);
title(['Required joint torques ',name])
ylabel('torque (Nm)');
xlabel('time (s)');
legend({namemap{:,1}},'Location','Best');
grid on;
print('-deps2c',['images/joint_torque',name]) 

tau_m = zeros(size(tau));
for i = 1:length(namemap(:,3))
    transmission = namemap{i,3};
    tau_m(:,i) = CSD_Tin(qd(:,i),30,tau(:,i),transmission);
end

motor_constant = [.255,.177,.177,.177,.121,.121,.121];
motor_constant = repmat(motor_constant,size(Tj,1),1);
%Pm = motor_power_heat(tau,r,transmission_efficiency);
Pm = (tau_m./motor_constant).^2;

figure(13)
plot(t,Pm)
title('Motor heat power')
ylabel('heat power (W)');
xlabel('time (s)');
legend({namemap{:,1}},'Location','Best');
grid on;

Pavg_motor = trapz(t,Pm,1)/max(t);
Pavg_motor_sum = sum(Pavg_motor);

[Pout,Pin,Ptransmissionloss] = transmission_power(tau,qd,transmission_efficiency);
Ptransmissionloss_avg = trapz(t,Ptransmissionloss,1)/max(t);
Ptransmissionloss_sum = sum(Ptransmissionloss_avg);

figure(14);
plot(t,Pout);
title(['Output power ',name])
legend({namemap{:,1}},'Location','Best');
grid on;


figure(15)
plot(t,[Pin+Pm,sum(Pin+Pm,2)])
title('Input power')
ylabel('power (W)');
xlabel('time (s)');
legend({namemap{:,1},'total'},'Location','Best');
grid on;
print('-deps2c',['images/input_power',name]) 


figure(16);
plot(t,Ptransmissionloss);
title('power transmission loss');
legend({namemap{:,1}},'Location','Best');

figure(17);
plot(t,qdd);
title('acceleration')
legend({namemap{:,1}},'Location','Best');


Pavg_motor_sum
Ptransmissionloss_sum
Pavg_motor_sum+Ptransmissionloss_sum