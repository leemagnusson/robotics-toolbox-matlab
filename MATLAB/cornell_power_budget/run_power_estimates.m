logfile_dir = 'logs/run' %'real_robot_03-25-16'
outfile_prefix = 'output/joint_torques_03-26-16_'
% Run robot with bed flat, robot tilted back 45 degrees, and robot tilted
% to the side 45 degrees:
gravities = -9.8 * [0, 0, sin(pi/2); cos(pi/4), 0, sin(pi/4); 0, cos(pi/4), sin(pi/4)]'

%%

startup_lee
verb_robot_2015
close all

%%

files = dir([logfile_dir, '/*.log']);   %# list all *.xyz files
files = {files.name}';                      %'# file names

fnames = fullfile(logfile_dir,files)     %# full path to file

%%

[results, allpwr] = estimate_log_power(r, other_param, gravities, outfile_prefix, fnames');
%[results, allpwr] = estimate_log_power(r, other_param, gravities, outfile_prefix, [{'STRAIGHTBACK_WIGGLE'}]);
results
maxpower = max(results.max_powers);
disp (['Overall max power: ', num2str(maxpower)]);
rmspower = sqrt(mean(allpwr.^2));
disp(['Overall rms power: ', num2str(rmspower)]);

figure
histogram(allpwr)
set(gca, 'YScale', 'log'); % Make histogram log scale
title('Mot power histogram over all logs/arms/gravity (log scale)');



