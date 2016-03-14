function [ t, q, eef, ws ] = import_logfile( logname , varargin)
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
ws = [-1,1,-1,1,-1,1];
eef_cols = 45:51;
t_tmp = data(:,2)-data(1,2);
qcols = [125,130,85,91,135,140,105,110,115,150,145];
switch logname
    case 'logs/hernia.log'
        data = data(t_tmp>34,:);
        qcols = [129,134,89,94,139,144,119,154,149];
        ws = [-.5,.5,-.1,.8,-.1,.7];
    case 'logs/lowerAnteria.log';
        qcols = [129,134,89,94,139,144,119,154,149];
        ws = [-.4,.4,-.5,.5,-.3,.8];
end

%%

if any(strcmp('selective',varargin))
    good_inds = 1:20:size(data,1);
else
    good_inds = 1:size(data,1);
end

ind = find(strcmp('interp_dt',varargin));
if ind
    interp_dt = varargin{ind+1};
else
    interp_dt = 1/1000;
end




q = data(good_inds,qcols);
if (size(q,2) == 11)
    q = [q(:,1:6),sum(q(:,7:9),2),q(:,10:end)];
end

eef = data(good_inds,eef_cols);
t = data(good_inds,2)-data(1,2);

ti = 0:interp_dt:max(t);

if any(strcmp('spline',varargin))
    eef = interp1(t,eef,ti,'spline');
    q = interp1(t,q,ti,'spline');
else % linear interpolation
    eef = interp1(t,eef,ti);
    q = interp1(t,q,ti);
end

t = ti;

ind = find(strcmp('smooth',varargin));
if ind
    %ti = 0:interp_dt:max(t);    % Not sure why this was here
    
    lowpass_cut = varargin{ind+1}; % Lowpas filter cutoff
    
    Wn = lowpass_cut/((1/interp_dt)/2);
    [b,a] = butter(4, Wn, 'low');
        
    qs = filtfilt(b, a, q);
    q = qs;

end

