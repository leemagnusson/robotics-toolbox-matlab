% CSD_Tin
% Andy Metzger
% 2015/01/08

function Tin = CSD_Tin(spd,tmp,Tout,trans)
% this function accepts the joint speed (rad/s), joint temperature (c),
% joint output torque (Nm), (eg CSD14_100),
% and outputs input torue (Nm)

% output from CSD_efficiency_coefficients
% a*T+b*T^2+c*theta_dot+d = etta

x25 = [1.1204,-0.0119,-0.0644,61.2143]; %CSD 20&25 with 100:1 ratio
x160 = [1.1208,0.0104,-0.0691,56.0014]; %CSD 20&25 with 160:1 ratio
x14 = [1.0897,-0.0104,-0.0671,54.1308]; %CSD 14 with 100:1 ratio


switch trans
    case 'CSD14_100'
        x = x14;
        n = 100;

    case 'CSD20_100'
        x = x25;
        n = 100;
        
    case 'CSD25_100'
        x = x25;
        n = 100;
        
    case 'CSD20_160'
        x = x160;
        n = 160;
        
    case 'CSD25_160'
        x = x160;
        n = 160;
        
    otherwise
        disp('invalid transmission name. Function expects one of the following:')
        disp('CSD14_100')
        disp('CSD20_100')
        disp('CSD25_100')
        disp('CSD20_160')
        disp('CSD25_160')
end

spd = spd*n;
etta = (x(1)*tmp + x(2)*tmp.^2 + x(3)*abs(spd) + x(4))/100;


Tin = zeros(size(Tout));
% quadrant 1 & 3
ind = (Tout >= 0 & spd >= 0) | (Tout <= 0 & spd <= 0);
Tin(ind) = Tout(ind)./etta(ind)/100;
% quadrant 2 & 4
ind = (Tout > 0 & spd < 0) | (Tout < 0 & spd > 0);
Tin(ind) = Tout(ind).*etta(ind)/100;
        
