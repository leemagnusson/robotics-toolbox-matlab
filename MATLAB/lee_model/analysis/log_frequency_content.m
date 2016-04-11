%% log_frequency_content.m 
% Lee Magnusson 3/10/16
% look at frequency content in log files.

%%
logname = 'logs/picknplace.log'

[t,q,eef,ws] = import_logfile(logname,'selective','spline');
[ts,qs,eefs] = import_logfile(logname,'selective','smooth');
[ta,qa,eefa,wsa] = import_logfile(logname);
[tp,qp,eefp] = import_logfile(logname,'selective');
dt = t(2)-t(1);

freq = fftfreq(t(2)-t(1),length(t));
%% eef, looking at different smoothing techniques
figure(1); clf;
plot(t,eef,ta,eefa,'.',tp,eefp,'*',ts,eefs);
title('eef position');

%% eef, spline, frequency
figure(2);
semilogx(freq,20*log10(abs(fft(eef))));
title('eef frequency');
xlabel('Hz')
grid on;

%% q, position, and freqency
figure(3); clf;
plot(t,q);
title('q');

figure(4); clf;
semilogx(freq,20*log10(abs(fft(q))));
title('q frequency');
xlabel('Hz');
grid on;

% note that the frequency goes smooth at 10 Hz only because of where I set
% the spline window. I'd say it's hard to interpret any specific drop off
% in frequency content.

