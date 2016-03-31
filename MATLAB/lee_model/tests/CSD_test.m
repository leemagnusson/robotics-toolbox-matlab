spd = linspace(-3500*2*pi/6000,3500*2*pi/6000,50);
To = linspace(-20,20,length(spd));


[spd_gr,To_gr] = meshgrid(spd,To);

T_in = CSD_Tin(spd_gr,30,To_gr,'CSD14_100')

clf
figure(1)
surf(spd_gr,To_gr,T_in)
xlabel('speed (rad/s)')
ylabel('output torque (Nm)')
zlabel('input torque (Nm)')
title('input torque as a function of joint speed and torque');