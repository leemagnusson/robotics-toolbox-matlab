%% Hernia measurement
% hernia setup from the measurement of CAD
%%
scale_mm_to_m = 1/1000; % 1000mm/1m
scale_deg_to_rad = pi/180;
translation_setup = [0;0;0]*scale_mm_to_m;
rotation_setup = RotationAxisAngle([1;0;0],pi/2);
artificial_offset = [0;0;0.05077];

% translation_target = rotation_setup * [-253.14; 1169.8; 7.19]*scale_mm_to_m+translation_setup + artificial_offset;
translation_target = rotation_setup * [-185.12; 1215.08; -26.29]*scale_mm_to_m+translation_setup + artificial_offset;
rotation_patient = rotation_setup;
translation_patient = translation_setup;

translation_trocar1 = rotation_setup * [-43.73;1388.79;128.51]*scale_mm_to_m+translation_setup + artificial_offset;
translation_trocar2 = rotation_setup * [19.66;1438;10.1]*scale_mm_to_m+translation_setup + artificial_offset;
translation_trocar3 = rotation_setup * [-43.73;1383.58;-149.32]*scale_mm_to_m+translation_setup + artificial_offset;
translation_trocar4 = rotation_setup * [-171.53;1350.29;-80.3]*scale_mm_to_m+translation_setup + artificial_offset;

translation_trocar = [translation_trocar1 translation_trocar2 translation_trocar3 translation_trocar4];

translation_camera = translation_target - 0.1 * (translation_target - translation_trocar2)/norm(translation_target - translation_trocar2);