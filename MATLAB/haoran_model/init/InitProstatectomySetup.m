%% Prostatectomy measurement
% The prostatectomy setup comes from the measurement of CAD model setup in a
% previous version of procedure setup. The number could be found in the
% excel file that document all the procedure setup numbers for trocar
% position, target position, table adapter number and etc. 
% https://docs.google.com/spreadsheets/d/18y37sFmPL6D-Hkzzqx7nnfmUNRatjIf0-TXWWM5oK2k/edit?ts=5716e4ab#gid=0
%%
scale_mm_to_m = 1/1000; % 1000mm/1m
scale_deg_to_rad = pi/180;
translation_setup = [0;0;0]*scale_mm_to_m;
rotation_setup = RotationAxisAngle([1;0;0],pi/2);


% translation_target = rotation_setup * [-253.14; 1169.8; 7.19]*scale_mm_to_m+translation_setup + artificial_offset;
translation_target = rotation_setup * [166.33; 1201.28; -1.88]*scale_mm_to_m+translation_setup;
rotation_patient = rotation_setup;
translation_patient = translation_setup;

translation_trocar1 = rotation_setup * [70.97;1459;96.08]*scale_mm_to_m+translation_setup;
translation_trocar2 = rotation_setup * [-23.63;1492.82;2.56]*scale_mm_to_m+translation_setup;
translation_trocar3 = rotation_setup * [70.9;1459;-96.07]*scale_mm_to_m+translation_setup;
translation_trocar4 = rotation_setup * [40.53;1428.57;-152.15]*scale_mm_to_m+translation_setup;

translation_trocar = [translation_trocar1 translation_trocar2 translation_trocar3 translation_trocar4];
translation_camera = translation_target - 0.1 * (translation_target - translation_trocar2)/norm(translation_target - translation_trocar2);
x_camera = (translation_camera - translation_trocar2)/norm(translation_camera - translation_trocar2);
vect_camera1 = (translation_trocar1 - translation_trocar2)/norm(translation_trocar1 - translation_trocar2);
z_camera = cross(vect_camera1,x_camera)/norm(cross(vect_camera1,x_camera));
y_camera = cross(z_camera,x_camera);
rotation_camera = [x_camera,y_camera,z_camera];

% Load hernia workspace model
vertex_prostatectomy_macro_workspace = stl2matlab('Prostatectomy_Workspace.STL');
scale_mm_to_m = 1/1000;
vertex_prostatectomy_macro_workspace{1} = vertex_prostatectomy_macro_workspace{1} * scale_mm_to_m;
vertex_prostatectomy_macro_workspace{2} = vertex_prostatectomy_macro_workspace{2} * scale_mm_to_m;
vertex_prostatectomy_macro_workspace{3} = vertex_prostatectomy_macro_workspace{3} * scale_mm_to_m;

for index = 1 : length(vertex_prostatectomy_macro_workspace{1})
    point_clouds_prostatectomy_workspace(:,index * 3 - 2) = rotation_setup * [vertex_prostatectomy_macro_workspace{1}(1,index);vertex_prostatectomy_macro_workspace{2}(1,index);vertex_prostatectomy_macro_workspace{3}(1,index)] + translation_setup;
    point_clouds_prostatectomy_workspace(:,index * 3 - 1) = rotation_setup * [vertex_prostatectomy_macro_workspace{1}(2,index);vertex_prostatectomy_macro_workspace{2}(2,index);vertex_prostatectomy_macro_workspace{3}(2,index)] + translation_setup;
    point_clouds_prostatectomy_workspace(:,index * 3 - 0) = rotation_setup * [vertex_prostatectomy_macro_workspace{1}(3,index);vertex_prostatectomy_macro_workspace{2}(3,index);vertex_prostatectomy_macro_workspace{3}(3,index)] + translation_setup;
end
save('..\data\point_clouds_prostatectomy_workspace.mat','point_clouds_prostatectomy_workspace');