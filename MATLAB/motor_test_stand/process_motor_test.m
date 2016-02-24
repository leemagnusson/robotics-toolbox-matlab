
%[rpm,brakeV,TmeasuredNm,busiA,busvVatdrive,motorphasephasevoltageVrms,iqA] ...
%    = import_motor_test2('test_data/motor test test data - 70-10 v2.csv');
[rpm,brakeV,iqA,TmeasuredNm,busiA,busvVatdrive] = import_motor_test('test_data/motor test test data - 70-10.csv',4,42);

r = .47*3/4+.16; 


npoles = 10;
n = length(rpm);
v = rpm*pi/30;
Pdrive = 2;

% current filter likely implemented on phases tf = 1/(tau*s+1)
current_filter = 20000;      % guess, Hz
tau = 1/current_filter/2/pi;
filter_mag = 1./(1+tau^2*(v*npoles*2*pi).^2);

iqA = iqA./filter_mag;

Pin = busiA.*busvVatdrive;
Pout_mech = v.*TmeasuredNm;

figure(1);
plot3(v,iqA,TmeasuredNm,'.');
grid on;
xlabel('v (rad/s)')
ylabel('iq')
zlabel('Tmeasured');

% try fitting in terms of power loss
A2 = [v,v.^2,iqA.^2, iqA ,ones(n,1),];
b2 = Pin-Pout_mech;
xp = A2\b2;

% linear regression for T + Tf + b*v = km*iq 
A = [iqA];
b = TmeasuredNm + xp(4) + xp(5)*v;
x = A\b;
x(2) = xp(4);
x(3) = xp(5);

% linear regression for T  = km*iq - Tf - b*v
A3 = [iqA -ones(n,1) -v];
b3 = TmeasuredNm;


% fit both power and torque
% Ploss = v*(Tf + b*v) + i^2*r + ks*V*i + Pdrive
% Tm = kt -b*v - Tf

W = diag([max(b3)*ones(n,1);max(b2)*ones(n,1)]);
A = [zeros(n,1),A2;
     A3,zeros(n,3)];
b = [b2;b3];
x = (W*A)\(W*b);

vg = linspace(0,400,100);
iqg = linspace(0,5,110);
[vgr,iqgr] = meshgrid(vg,iqg);
Tgr = x(1)*iqgr - x(2) - x(3)*vgr;
figure(2); clf; hold on;
surf(vgr,iqgr,Tgr);
plot3(v,iqA,TmeasuredNm,'.');
grid on;
xlabel('v (rad/s)')
ylabel('iq')
zlabel('Tmeasured');


figure(3);
title('efficiency')
plot3(v,iqA,Pout_mech./Pin,'.');

figure(4);
title('power in')
plot3(v,iqA,Pin,'.');

figure(5);
title('power loss')
plot3(v,iqA,Pin-Pout_mech,'.');



Pout_mech_gr = Tgr.*vgr;
Ploss_motor_gr = Pdrive + iqgr.^2*r + vgr*x(2) + vgr.^2*.000075;% vgr.^2*x(3)+ vgr.^3*x(4)+;
Ploss_motor_gr = x(6) + x(5)*iqgr + x(4)*iqgr.^2 + x(2)*vgr + x(3)*vgr.^2;
figure(6); clf; hold on;
plot3(v,iqA,Pin-Pout_mech,'.');
surf(vgr,iqgr,Ploss_motor_gr);


% convert line-line calc phase amplitude
Vmotor = x(1)*2/sqrt(3)*v+iqA*2/sqrt(3)*x(4);

