function [arm] = import_logfile_eef( logname,step_size)
%IMPORT_LOGFILE Read Verb log files
%   preparsed to only include new data points
%   q, cmd of [should_p, should_r, elbow, forearm, spher_b, spher_r,
%   pit_a+pit_b+pit_c, transl, rotate]
%
%   Options:
%       'selective' - Only use points where trajectory changed
%       'interp_dt' <dt> - Specify the dt to use for interpolation
%                           (otherwise uses 1kHz default)
%       'spline' - Do spine interpolation between points
%       'smooth' <cutoff> - Smooth q with a lowpass filter with <cutoff>
%                           cutoff frequency in Hz


data = import_log(logname);
num = length(data);
arm0 = data(1:step_size:num,15:21);
arm1 = data(1:step_size:num,170:176);
arm2 = data(1:step_size:num,325:331);
arm3 = data(1:step_size:num,480:486);

% arm0 = data(1:step_size:num,45:51);
% arm1 = data(1:step_size:num,196:202);
% arm2 = data(1:step_size:num,347:353);
% arm3 = data(1:step_size:num,498:504);

% arm0 = data(1:step_size:num,25:31);
% arm1 = data(1:step_size:num,176:182);
% arm2 = data(1:step_size:num,327:333);
% arm3 = data(1:step_size:num,478:484);

% arm0 = data(1:step_size:num,65:71);
% arm1 = data(1:step_size:num,216:222);
% arm2 = data(1:step_size:num,367:373);
% arm3 = data(1:step_size:num,518:524);

arm(:,:,1) = arm0;
arm(:,:,2) = arm1;
arm(:,:,3) = arm2;
arm(:,:,4) = arm3;
end

