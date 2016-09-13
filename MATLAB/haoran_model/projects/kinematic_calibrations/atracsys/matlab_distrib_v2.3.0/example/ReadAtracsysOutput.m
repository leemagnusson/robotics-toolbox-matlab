function [t,x,y,z,err] = ReadAtracsysOutput(filename)
% read output from ftk7_AcquisitionExtendedCMAKE 2016-06-22
% DRM 2016-07-06 moved timestamp to first column
fileID = fopen(filename);
%               t  x  y  z r1 r2 r3 r4 r5 r6 r7 r8 r9 er
formatSpec = ' %f %f %f %f %f %f %f %f %f %f %f %f %f %f';
cell = textscan(fileID,formatSpec,'Delimiter',';');
fclose(fileID);

t = cell{1}/1e6; % timestamp in seconds
t = t - t(1); % start from t=0

x = cell{2}/1e3; % in meters
y = cell{3}/1e3; %
z = cell{4}/1e3; %
% rotation parameters are given row-wise
R11 = cell{5};
R12 = cell{6};
R13 = cell{7};
R21 = cell{8};
R22 = cell{9};
R23 = cell{10};
R31 = cell{11};
R32 = cell{12};
R33 = cell{13};

err = cell{14}; %

for i=1:length(R11)
    R(1,1,i) = R11(i);
    R(1,2,i) = R12(i);
    R(1,3,i) = R13(i);
    R(2,1,i) = R21(i);
    R(2,2,i) = R22(i);
    R(2,3,i) = R23(i);
    R(3,1,i) = R31(i);
    R(3,2,i) = R32(i);
    R(3,3,i) = R33(i);
end

speed_capture = length(t)/t(end);

figure
n = 4+1;
subplot(n,1,1)
plot(t,x); ylabel('x [m]');
xlabel('t [s]'); grid minor; axis tight
title(['amplitude [mm]: ',num2str((max(x)-mean(x))*1000)])

subplot(n,1,2)
plot(t,y); ylabel('y [m]');
xlabel('t [s]'); grid minor; axis tight
title(['amplitude [mm]: ',num2str((max(y)-mean(y))*1000)])

subplot(n,1,3)
plot(t,z); ylabel('z [m]');
xlabel('t [s]'); grid minor; axis tight
title(['amplitude [mm]: ',num2str((max(z)-mean(z))*1000)])

subplot(n,1,4)
plot(t,err); ylabel('error [mm]');
xlabel('t [s]'); grid minor; axis tight

subplot(n,1,5)
t_delta = t(2:end)-t(1:end-1);
plot(t(2:end),1./t_delta);  ylabel('capture speed [Hz]');
xlabel('t [s]'); grid minor; axis tight
title(['mean capture speed: ',num2str(round(speed_capture)),' Hz'])
end

