%% LAR measurement 1
% The LAR setup 1 comes from the measurement of CAD model setup in a
% previous version of procedure setup. The number could be found in the
% excel file that document all the procedure setup numbers for trocar
% position, target position, table adapter number and etc. 
% https://docs.google.com/spreadsheets/d/18y37sFmPL6D-Hkzzqx7nnfmUNRatjIf0-TXWWM5oK2k/edit?ts=5716e4ab#gid=0
%%
scale_mm_to_m = 1/1000; % 1000mm/1m
scale_deg_to_rad = pi/180;
translation_setup = [0;0;0]*scale_mm_to_m;
rotation_setup = RotationAxisAngle([1;0;0],pi/2);

translation_target = rotation_setup * [124.02;1258.29;0]*scale_mm_to_m+translation_setup;
rotation_patient = rotation_setup;
translation_patient = translation_setup;

translation_trocar1 = rotation_setup * [41.51;1469.88;-88.39]*scale_mm_to_m+translation_setup;
translation_trocar2 = rotation_setup * [-36.08;1490.27;6.37]*scale_mm_to_m+translation_setup;
translation_trocar3 = rotation_setup * [60.07;1439.52;132.65]*scale_mm_to_m+translation_setup;
translation_trocar4 = rotation_setup * [-97.85;1431.89;113.7]*scale_mm_to_m+translation_setup;

translation_trocar = [translation_trocar1 translation_trocar2 translation_trocar3 translation_trocar4];
translation_camera = translation_target - 0.1 * (translation_target - translation_trocar2)/norm(translation_target - translation_trocar2);