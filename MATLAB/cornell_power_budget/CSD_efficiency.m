
speed = [500,1000,2000,3500];
temp = -10:5:40;

%CSD-25-100 & CSD-20-100
n_csd25_500 =  [52,57,63,68,72,74,76,78,79,80,81];
n_csd25_1000 = [42,48,54,60,65,68,71,73,75,77,78];
n_csd25_2000 = [31,37,44,50,55,60,64,67,70,72,74];
n_csd25_3500 = [24,30,36,42,47,53,57,61,64,67,69];

%a*T+b*T^2+c*theta_dot+d = etta
A25 = [temp',speed(1)*ones(length(temp),1),(temp.^2)',ones(length(temp),1);...
    temp',speed(2)*ones(length(temp),1),(temp.^2)',ones(length(temp),1);...
    temp',speed(3)*ones(length(temp),1),(temp.^2)',ones(length(temp),1);...
    temp',speed(4)*ones(length(temp),1),(temp.^2)',ones(length(temp),1)];

B25 = [n_csd25_500';n_csd25_1000';n_csd25_2000';n_csd25_3500'];

x25=A25\B25

figure(1); clf
plot3(A25(:,1),A25(:,2),B25,'bo');
hold all

plot3(A25(:,1),A25(:,2),A25*x25,'r*')
grid


%CSD-14-100
n_csd14_500 =  [43,50,56,61,64,67,69,72,73,74,75];
n_csd14_1000 = [34,40,46,52,57,61,64,66,68,71,72];
n_csd14_2000 = [25,30,37,42,48,52,56,59,62,65,67];
n_csd14_3500 = [18,23,28,34,40,44,48,52,56,59,62];

%a*T+b*T^2+c*theta_dot+d = etta
A14 = [temp',speed(1)*ones(length(temp),1),(temp.^2)',ones(length(temp),1);...
    temp',speed(2)*ones(length(temp),1),(temp.^2)',ones(length(temp),1);...
    temp',speed(3)*ones(length(temp),1),(temp.^2)',ones(length(temp),1);...
    temp',speed(4)*ones(length(temp),1),(temp.^2)',ones(length(temp),1)];

B14 = [n_csd14_500';n_csd14_1000';n_csd14_2000';n_csd14_3500'];

x14=A14\B14

% figure(1); clf
% plot3(A14(:,1),A14(:,2),B14,'bo');
% hold all
% 
% plot3(A14(:,1),A14(:,2),A14*x14,'r*')
% grid

tmp_0 = linspace(-10,40,50);
spd_0 = linspace(0,3500,length(tmp_0));
etta = ones(length(tmp_0));

% for i=1:length(tmp_0)
%     etta(i,:) = x25(1)*tmp_0(i) + x25(2)*spd_0 + x25(3)*(tmp_0(i)).^2 + x25(4);
% end

[temp_gr,spd_gr] = meshgrid(tmp_0,spd_0);
etta = x25(1)*temp_gr + x25(2)*spd_gr + x25(3)*temp_gr.^2 + x25(4);
figure(2)
surf(temp_gr,spd_gr,etta)
xlabel('temperature (C)')
ylabel('speed (rpm)')
zlabel('efficiency')




