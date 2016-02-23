
%[rpm,brakeV,TmeasuredNm,busiA,busvVatdrive,motorphasephasevoltageVrms,iqA] ...
%    = import_motor_test2('test_data/motor test test data - 70-10 v2.csv');
[rpm,brakeV,iqA,TmeasuredNm,busiA,busvVatdrive] = import_motor_test('test_data/motor test test data - 70-10.csv',4,38);

r = .47; % note wrong units need to fix

n = length(rpm);
v = rpm*pi/30;

Pin = busiA.*busvVatdrive;
Pout_mech = v.*TmeasuredNm;

figure(1);
plot3(v,iqA,TmeasuredNm,'.');
grid on;
xlabel('v (rad/s)')
ylabel('iq')
zlabel('Tmeasured');

% linear regression for T = km*iq - Tf - b*v
A = [iqA,-ones(n,1),-v,-v.^2];
b = TmeasuredNm;
x = A\b;

vg = linspace(0,400,100);
iqg = linspace(0,5,110);
[vgr,iqgr] = meshgrid(vg,iqg);
Tgr = x(1)*iqgr - x(2) - x(3)*vgr - x(4)*vgr.^2;
figure(2); clf; hold on;
surf(vgr,iqgr,Tgr);
plot3(v,iqA,TmeasuredNm,'.');
grid on;
xlabel('v (rad/s)')
ylabel('iq')
zlabel('Tmeasured');


figure(3);
plot3(v,iqA,Pout_mech./Pin,'.');

figure(4);
plot3(v,iqA,Pin,'.');

figure(5);
plot3(v,iqA,Pin-Pout_mech,'.');

Pout_mech_gr = Tgr.*vgr;
Ploss_motor_gr = iqgr.^2*r + Tgr*x(2) + vgr.^2*x(3)+ vgr.^3*x(4);
figure(6); clf; hold on;
plot3(v,iqA,Pin-Pout_mech,'.');
%surf(vgr,iqgr,Ploss_motor_gr);
