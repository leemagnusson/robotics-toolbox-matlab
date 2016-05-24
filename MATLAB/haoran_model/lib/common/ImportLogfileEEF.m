function [arm_eef,time] = ImportLogfileEEF( name_log,step_size)
%IMPORT_LOGFILE Read Verb log files
%   This function takes the raw log data and import only the eef data for
%   all four arms.
% [arm_eef,time] = ImportLogfileEEF( name_log,step_size)
% Input:
% name_log: name string of the log
% step_size: sample size of the log import. step_size = 10 means load data for every 10
% samples.
% arm_eef: n by 7 by 4 matrix. first index is for sample index, second
% index is for quaternion and the third index is for arm number
% time: n by 1 vector with time starting from 0.
% please double check the column numbers for a new log.


data = ImportLog(name_log);
num = length(data);
time = data(1:step_size:num,2) - data(1,2);
% hernia.log
% arm0 = data(1:step_size:num,15:21);
% arm1 = data(1:step_size:num,170:176);
% arm2 = data(1:step_size:num,325:331);
% arm3 = data(1:step_size:num,480:486);

% suture_3.log
arm0 = data(1:step_size:num,15:21);
arm1 = data(1:step_size:num,166:172);
arm2 = data(1:step_size:num,317:323);
arm3 = data(1:step_size:num,468:474);

arm_eef(:,:,1) = arm0;
arm_eef(:,:,2) = arm1;
arm_eef(:,:,3) = arm2;
arm_eef(:,:,4) = arm3;
end

