scale = 1000;
deg2rad = pi/180;
setup_offset = [0;0;-1093]/scale;
R_setup = RotationMatrix_rad(pi/2,[1;0;0]);
Hernia = R_setup * [34.9827;1499.0355;0]/scale+setup_offset;

R_Hernia_Body = R_setup * RotationMatrix_rad(pi,[0;1;0]);
Hernia_Body = R_setup * [-15.8173;1143.4355;0]/scale + setup_offset + [0.45;-0.35;-0.07];

Trocar1 = R_setup * [63.1933;1389.8945;-186.0630]/scale+setup_offset;
Trocar2 = R_setup * [-3.4349;1388.7323;-193.9687]/scale+setup_offset;
Trocar3 = R_setup * [-69.1751;1391.3410;-180.5226]/scale+setup_offset;
Trocar = [Trocar1 Trocar2 Trocar3];
Base1 = R_setup * [147.2504;1093.9055;379.6100]/scale+setup_offset;
R_Base1 = R_setup * RotationMatrix_rad(pi,[0;0;1])*RotationMatrix_rad(pi/2,[1;0;0]);
T_Base1 = [R_Base1 Base1; 0 0 0 1];
Base2 = R_setup * [455.9674;1093.9055;-490.3583]/scale+setup_offset;
R_Base2 = R_setup * RotationMatrix_rad(-pi/2,[1;0;0])*RotationMatrix_rad(-40*pi/180,[0;0;1]);
T_Base2 = [R_Base2 Base2; 0 0 0 1];
Base3 = R_setup * [-338.7620;1089.7892;-379.6100]/scale+setup_offset;
% Base3 = Base3 + [-0.17;0.1;0];
R_Base3 = R_setup * RotationMatrix_rad(-pi/2,[1;0;0]);
% R_Base3 = R_Base3 * RotationMatrix_rad(pi/4,[0;0;1]);
T_Base3 = [R_Base3 Base3; 0 0 0 1];
base_T_setup(:,:,1) = T_Base1;
base_T_setup(:,:,2) = T_Base2;
base_T_setup(:,:,3) = T_Base3;
p1_o = R_setup * [312.3535;1417.5403;-63.6754]/scale+setup_offset;
p2_o = R_setup * [198.1567;1543.8298;-308.5523]/scale+setup_offset;
p3_o = R_setup * [-74.9742;1408.4679;-458.9052]/scale+setup_offset;
p1_x = R_setup * [290.9767;1426.9616;-22.2840]/scale+setup_offset;
p1_z = R_setup * [297.0597;1415.8718;-71.1942]/scale+setup_offset;
p2_x = R_setup * [211.1218;1558.8781;-265.3733]/scale+setup_offset;
p2_z = R_setup * [185.7641;1534.3322;-301.5212]/scale+setup_offset;
p3_x = R_setup * [-110.4403;1376.8503;-460.1116]/scale+setup_offset;
p3_z = R_setup * [-74.5988;1407.3949;-441.8194]/scale+setup_offset;
x_Trocar1 = (p1_x-p1_o)./norm(p1_x-p1_o);
x_Trocar2 = (p2_x-p2_o)./norm(p2_x-p2_o);
x_Trocar3 = (p3_x-p3_o)./norm(p3_x-p3_o);

z_Trocar1 = (p1_z-p1_o)./norm(p1_z-p1_o);
z_Trocar2 = (p2_z-p2_o)./norm(p2_z-p2_o);
z_Trocar3 = (p3_z-p3_o)./norm(p3_z-p3_o);

y_Trocar1 = cross(z_Trocar1,x_Trocar1)./norm(cross(z_Trocar1,x_Trocar1));
y_Trocar2 = cross(z_Trocar2,x_Trocar2)./norm(cross(z_Trocar2,x_Trocar2));
y_Trocar3 = cross(z_Trocar3,x_Trocar3)./norm(cross(z_Trocar3,x_Trocar3));

z_Trocar1 = cross(x_Trocar1,y_Trocar1);
z_Trocar2 = cross(x_Trocar2,y_Trocar2);
z_Trocar3 = cross(x_Trocar3,y_Trocar3);

R_Trocar1 = [x_Trocar1 y_Trocar1 z_Trocar1];
R_Trocar1 = R_Trocar1 * RotationMatrix_rad(pi/18,[1;0;0]) * RotationMatrix_rad(-pi/18,[0;1;0]);
R_Trocar2 = [x_Trocar2 y_Trocar2 z_Trocar2];
R_Trocar3 = [x_Trocar3 y_Trocar3 z_Trocar3];
R_Trocar3 = R_Trocar3 * RotationMatrix_rad(-2*pi/3,[0;0;1]) * RotationMatrix_rad(-pi/6,[0;1;0]) * RotationMatrix_rad(-pi/3,[0;0;1]);

R_Trocar(:,:,1) = R_Trocar1;
R_Trocar(:,:,2) = R_Trocar2;
R_Trocar(:,:,3) = R_Trocar3;

% q1_set = [-70.85;25.5;16.48;-18.09;53;-185.95;0;0;0;0;0]*deg2rad;
% q2_set = [-5.97;-0.68;-31.68;18.75;66.62;-121.21;0;0;0;0;0]*deg2rad;
% q3_set = [26.81;-42.17;-16.42;-85.22;80.36;-9.89;0;0;0;0;0]*deg2rad;
q1_set = [30;-45;0;90;30;0;0;0;0;0;0]*deg2rad;
q2_set = [0;0;0;-20;0;100;0;0;0;0;0]*deg2rad;
q3_set = [-45;45;0;0;45;180;-45;0;0;0;0]*deg2rad;
q_set = [q1_set q2_set q3_set];
