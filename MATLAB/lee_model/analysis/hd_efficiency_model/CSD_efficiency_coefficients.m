% CSD_efficiency_coefficients
% Andy Metzger
% 2015/01/08

%input speed converted to rad/s
speed = [500,1000,2000,3500]*(2*pi)/60;

%CSD temperature in C
temp = -10:5:40;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               %
%   CSD-25-100 & CSD-20-100     %
%                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_csd25_500 =  [52,57,63,68,72,74,76,78,79,80,81];
n_csd25_1000 = [42,48,54,60,65,68,71,73,75,77,78];
n_csd25_2000 = [31,37,44,50,55,60,64,67,70,72,74];
n_csd25_3500 = [24,30,36,42,47,53,57,61,64,67,69];

%a*T+b*T^2+c*theta_dot+d = etta
A25 = [temp',(temp.^2)',speed(1)*ones(length(temp),1),ones(length(temp),1);...
       temp',(temp.^2)',speed(2)*ones(length(temp),1),ones(length(temp),1);...
       temp',(temp.^2)',speed(3)*ones(length(temp),1),ones(length(temp),1);...
       temp',(temp.^2)',speed(4)*ones(length(temp),1),ones(length(temp),1)];

B25 = [n_csd25_500';n_csd25_1000';n_csd25_2000';n_csd25_3500'];

x25=A25\B25

figure(1); clf
plot3(A25(:,1),A25(:,3),B25,'bo');
hold all

plot3(A25(:,1),A25(:,3),A25*x25,'r*')
grid
xlabel('temperature (C)');
ylabel('speed (rad/s)');
zlabel('efficiency')
title('CSD-20 & CSD-25 100:1 efficiency from catalog');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               %
%   CSD-25-160 & CSD-20-160     %
%                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_csd160_500 =  [45,52,57,62,67,70,72,74,76,77,78];
n_csd160_1000 = [35,42,48,54,60,63,66,69,71,73,75];
n_csd160_2000 = [25,31,37,43,49,54,57,61,65,67,70];
n_csd160_3500 = [19,25,31,35,41,45,50,54,58,62,64];

%a*T+b*T^2+c*theta_dot+d = etta
A160 = [temp',(temp.^2)',speed(1)*ones(length(temp),1),ones(length(temp),1);...
        temp',(temp.^2)',speed(2)*ones(length(temp),1),ones(length(temp),1);...
        temp',(temp.^2)',speed(3)*ones(length(temp),1),ones(length(temp),1);...
        temp',(temp.^2)',speed(4)*ones(length(temp),1),ones(length(temp),1)];

B160 = [n_csd160_500';n_csd160_1000';n_csd160_2000';n_csd160_3500'];

x160=A160\B160

figure(2); clf
plot3(A160(:,1),A160(:,3),B160,'bo');
hold all

plot3(A160(:,1),A160(:,3),A160*x160,'r*')
grid
xlabel('temperature (C)');
ylabel('speed (rad/s)');
zlabel('efficiency')
title('CSD-20 & CSD-25 160:1 efficiency from catalog');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               %
%           CSD-14-100          %
%                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_csd14_500 =  [43,50,56,61,64,67,69,72,73,74,75];
n_csd14_1000 = [34,40,46,52,57,61,64,66,68,71,72];
n_csd14_2000 = [25,30,37,42,48,52,56,59,62,65,67];
n_csd14_3500 = [18,23,28,34,40,44,48,52,56,59,62];

%a*T+b*T^2+c*theta_dot+d = etta
A14 = [temp',(temp.^2)',speed(1)*ones(length(temp),1),ones(length(temp),1);...
       temp',(temp.^2)',speed(2)*ones(length(temp),1),ones(length(temp),1);...
       temp',(temp.^2)',speed(3)*ones(length(temp),1),ones(length(temp),1);...
       temp',(temp.^2)',speed(4)*ones(length(temp),1),ones(length(temp),1)];

B14 = [n_csd14_500';n_csd14_1000';n_csd14_2000';n_csd14_3500'];

x14=A14\B14

figure(3); clf
plot3(A14(:,1),A14(:,3),B14,'bo');
hold all

plot3(A14(:,1),A14(:,3),A14*x14,'r*')
grid
xlabel('temperature (C)');
ylabel('speed (rad/s)');
zlabel('efficiency')
title('CSD-14 100:1 efficiency from catalog');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                       %
%   surface plots from fit equations    %
%                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tmp_0 = linspace(-10,40,50);
spd_0 = linspace(0,3500*2*pi/60,length(tmp_0));
etta = ones(length(tmp_0));

[temp_gr,spd_gr] = meshgrid(tmp_0,spd_0);

%CSD-25-100 & CSD-20-100
etta25 = x25(1)*temp_gr + x25(2)*temp_gr.^2 + x25(3)*spd_gr + x25(4);
figure(4)
surf(temp_gr,spd_gr,etta25)
xlabel('temperature (C)')
ylabel('speed (rad/s)')
zlabel('efficiency')
title('CSD-20 & CSD-25 100:1 efficiency');

%CSD-25-160 & CSD-20-160
etta160 = x160(1)*temp_gr + x160(2)*temp_gr.^2 + x160(3)*spd_gr + x160(4);
figure(5)
surf(temp_gr,spd_gr,etta160)
xlabel('temperature (C)')
ylabel('speed (rad/s)')
zlabel('efficiency')
title('CSD-20 & CSD-25 160:1 efficiency');

%CSD-14-100
etta14 = x14(1)*temp_gr + x14(2)*temp_gr.^2 + x14(3)*spd_gr + x14(4);
figure(6)
surf(temp_gr,spd_gr,etta14)
xlabel('temperature (C)')
ylabel('speed (rad/s)')
zlabel('efficiency')
title('CSD-14 100:1 efficiency');




