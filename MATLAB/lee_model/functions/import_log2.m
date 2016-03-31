%%
logname = 'logs/lowerAnteria.log';
%logname = 'logs/hernia.log';
logname = 'logs/picknplace.log';

data = import_log(logname);
ws = [-1,1,-1,1,-1,1];

t_tmp = data(:,2)-data(1,2);
switch logname
    case 'logs/hernia.log'
        data = data(t_tmp>34,:);
        ws = [-.5,.5,-.1,.8,-.1,.7];
    case 'logs/lowerAnteria.log';
        ws = [-.4,.4,-.5,.5,-.3,.8];
end

%%

qm = data(:,[129,134,89,94,139,144,119,154,149]);
%qm(:,5) = -qm(:,5);
%qm(:,4) = -qm(:,4);

tm = data(:,2)-data(1,2);
dt = 1/3000;
t = 0:dt:max(tm);
q = interp1(tm,qm,t);
qs = idealfilt(t,q,1000,'lowpass',2);
q = qs;
qd = [zeros(1,size(q,2));diff(q,1,1)]/dt;
qdd = [zeros(1,size(q,2));diff(q,2,1);zeros(1,size(q,2))]/dt^2;

figure(1);clf; hold all;
plot(t,q);
plot(tm,qm,'.');
%r.plot3d(qi(300:end,:),'workspace',[-.5,.5,-1,1,-.1,1]);


figure(2); clf; hold all;
plot(tm,qm,'.')
plot(t,qs);
title('q')

figure(3);
plot(t,qd);
title('qd');

figure(4)
plot(t,qdd);
title('qdd');
%% Inverse dynamics 

T = r.rne(q,qd,qdd);

figure(10); clf;
plot(t,T);
grid on;
legend(other_param.name1,'location','best');
accel = max(abs(T),[],1) - abs(mean(T))
title('joint torque');

%% plot robot
movie = false
fname = logname
do_plot = false

tm = 0:1/30:max(t);
qm = interp1(t,q,tm);

figure(20);
if do_plot
    if movie
        r.plot3d(qm,'workspace',ws,'movie','movie');
        system(['ffmpeg -r 30 -i movie/%04d.png -c:v libx264 -preset slow -tune animation -crf 22 ', fname,'.mkv']);
        
    else 
        r.plot3d(qm,'workspace',ws);
    end
end
    
    
 %%
 
 [Pm,Pt] = motor_power_heat(T,qd,other_param);
Pm_sum = sum(Pm,2);
figure(19); clf; hold on;
plot(t,Pm,t,Pm_sum);

figure(20); clf; hold on;
plot(t,Pt,t,sum(Pt,2));

Pm_mean = mean(sum(Pm,2))
Pt_mean = mean(sum(Pt,2))