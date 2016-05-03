file = '../../../../arm_pwr_budget/real_robot_data/fast_spherical1.csv';
mvperamp = 500;
voltage = 48;

[filepath, filename, fileext] = fileparts(file);

[t,i] = import_scope_current(file,mvperamp);

p = i * voltage;

rmspower = sqrt(mean(p.^2))
peakpower = max(p)

h = ones(1,10)/10;
psmooth = filter(h,1,p);

figure;
hold on;

subplot(2,1,1);
hold on;
plot(t,p);
plot(t,psmooth);
xlabel('Time (Seconds)');
ylabel('Power (Watts)');
title([filename, ' Power over time']);

subplot(2,1,2);
histogram(p);
xlabel('Power (Watts)');
ylabel('Num Samples');
title([filename, ' Power Histogram']);