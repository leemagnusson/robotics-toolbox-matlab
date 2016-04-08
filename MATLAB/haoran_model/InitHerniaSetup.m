%% Hernia measurement
% hernia setup from the measurement of CAD
%%
scale_mm_to_m = 1/1000; % 1000mm/1m
scale_deg_to_rad = pi/180;
translation_setup = [0;0;0]*scale_mm_to_m;
rotation_setup = RotationAxisAngle([1;0;0],pi/2);

translation_hernia = rotation_setup * [34.9827;1499.0355;0]*scale_mm_to_m+translation_setup;
rotation_hernia_patient = rotation_setup * RotationAxisAngle([0;1;0],pi);
% Hernia_Body = R_setup * [-15.8173;1143.4355;0]/scale + setup_offset + [0.45;-0.35;-0.07];
translation_hernia_patient = rotation_setup * [-15.8173;1143.4355;0]*scale_mm_to_m + translation_setup;

translation_trocar1 = rotation_setup * [63.1933;1389.8945;-186.0630]*scale_mm_to_m+translation_setup;
translation_trocar2 = rotation_setup * [-3.4349;1388.7323;-193.9687]*scale_mm_to_m+translation_setup;
translation_trocar3 = rotation_setup * [-69.1751;1391.3410;-180.5226]*scale_mm_to_m+translation_setup;
translation_trocar = [translation_trocar1 translation_trocar2 translation_trocar3];

translation_base1 = rotation_setup * [147.2504;1093.9055;379.6100]*scale_mm_to_m+translation_setup;
rotation_base1 = rotation_setup * RotationAxisAngle([0;0;1],pi)*RotationAxisAngle([1;0;0],pi/2);
transformation_base1 = [rotation_base1 translation_base1; 0 0 0 1];
translation_base2 = rotation_setup * [455.9674;1093.9055;-490.3583]*scale_mm_to_m+translation_setup;
rotation_base2 = rotation_setup * RotationAxisAngle([1;0;0],-pi/2)*RotationAxisAngle([0;0;1],-40*pi/180);
transformation_base2 = [rotation_base2 translation_base2; 0 0 0 1];
translation_base3 = rotation_setup * [-338.7620;1089.7892;-379.6100]*scale_mm_to_m+translation_setup;
rotation_base3 = rotation_setup * RotationAxisAngle([1;0;0],-pi/2);
transformation_base3 = [rotation_base3 translation_base3; 0 0 0 1];
transformation_base(:,:,1) = transformation_base1;
transformation_base(:,:,2) = transformation_base2;
transformation_base(:,:,3) = transformation_base3;

p1_o = rotation_setup * [312.3535;1417.5403;-63.6754]*scale_mm_to_m+translation_setup;
p2_o = rotation_setup * [198.1567;1543.8298;-308.5523]*scale_mm_to_m+translation_setup;
p3_o = rotation_setup * [-74.9742;1408.4679;-458.9052]*scale_mm_to_m+translation_setup;
p1_x = rotation_setup * [290.9767;1426.9616;-22.2840]*scale_mm_to_m+translation_setup;
p1_z = rotation_setup * [297.0597;1415.8718;-71.1942]*scale_mm_to_m+translation_setup;
p2_x = rotation_setup * [211.1218;1558.8781;-265.3733]*scale_mm_to_m+translation_setup;
p2_z = rotation_setup * [185.7641;1534.3322;-301.5212]*scale_mm_to_m+translation_setup;
p3_x = rotation_setup * [-110.4403;1376.8503;-460.1116]*scale_mm_to_m+translation_setup;
p3_z = rotation_setup * [-74.5988;1407.3949;-441.8194]*scale_mm_to_m+translation_setup;
x_trocar1 = (p1_x-p1_o)./norm(p1_x-p1_o);
x_trocar2 = (p2_x-p2_o)./norm(p2_x-p2_o);
x_trocar3 = (p3_x-p3_o)./norm(p3_x-p3_o);

z_trocar1 = (p1_z-p1_o)./norm(p1_z-p1_o);
z_trocar2 = (p2_z-p2_o)./norm(p2_z-p2_o);
z_trocar3 = (p3_z-p3_o)./norm(p3_z-p3_o);

y_trocar1 = cross(z_trocar1,x_trocar1)./norm(cross(z_trocar1,x_trocar1));
y_trocar2 = cross(z_trocar2,x_trocar2)./norm(cross(z_trocar2,x_trocar2));
y_trocar3 = cross(z_trocar3,x_trocar3)./norm(cross(z_trocar3,x_trocar3));

z_trocar1 = cross(x_trocar1,y_trocar1);
z_trocar2 = cross(x_trocar2,y_trocar2);
z_trocar3 = cross(x_trocar3,y_trocar3);

rotation_trocar1 = [x_trocar1 y_trocar1 z_trocar1];
rotation_trocar1 = rotation_trocar1 * RotationAxisAngle([1;0;0],pi/18) * RotationAxisAngle([0;1;0],-pi/18);
rotation_trocar2 = [x_trocar2 y_trocar2 z_trocar2];
rotation_trocar3 = [x_trocar3 y_trocar3 z_trocar3];
rotation_trocar3 = rotation_trocar3 * RotationAxisAngle([0;0;1],-2*pi/3) * RotationAxisAngle([0;1;0],-pi/6) * RotationAxisAngle([0;0;1],-pi/3);

rotation_trocar(:,:,1) = rotation_trocar1;
rotation_trocar(:,:,2) = rotation_trocar2;
rotation_trocar(:,:,3) = rotation_trocar3;

translation_camera = translation_hernia - 0.1 * (translation_hernia - translation_trocar2)/norm(translation_hernia - translation_trocar2);
x_camera = (translation_camera - translation_trocar2)/norm(translation_camera - translation_trocar2);
vect_camera1 = (translation_trocar1 - translation_trocar2)/norm(translation_trocar1 - translation_trocar2);
z_camera = cross(vect_camera1,x_camera)/norm(cross(vect_camera1,x_camera));
y_camera = cross(z_camera,x_camera);
rotation_camera = [x_camera,y_camera,z_camera];
rotation_camera = rotation_camera * RotationAxisAngle([1;0;0],pi);

q1_set = [30;-45;0;90;30;0;0;0;0;0;0]*scale_deg_to_rad;
q2_set = [0;0;0;-20;0;100;0;0;0;0;0]*scale_deg_to_rad;
q3_set = [-45;20;-10;0;45;180;-45;0;0;0;0]*scale_deg_to_rad;
q_set = [q1_set q2_set q3_set];
