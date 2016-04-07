function [arm_eef,time] = ImportLogfileEEF( name_log,step_size)
%IMPORT_LOGFILE Read Verb log files
%   This function takes the raw log data and import only the eef data for
%   all four arms.
% [arm_eef,time] = ImportLogfileEEF( name_log,step_size)
% please double check the column numbers for a new log.


data = ImportLog(name_log);
num = length(data);
time = data(1:step_size:num,2) - data(1,2);
arm0 = data(1:step_size:num,15:21);
arm1 = data(1:step_size:num,170:176);
arm2 = data(1:step_size:num,325:331);
arm3 = data(1:step_size:num,480:486);

arm_eef(:,:,1) = arm0;
arm_eef(:,:,2) = arm1;
arm_eef(:,:,3) = arm2;
arm_eef(:,:,4) = arm3;
end

