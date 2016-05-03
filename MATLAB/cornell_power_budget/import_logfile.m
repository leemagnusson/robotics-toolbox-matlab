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
%       'arm' <armnum> = Which arm to import. If unspecified, uses arm0

logfile = fopen(logname, 'r');
if logfile == -1
    display('Failed to open ', logname);
    return;
end

% See which arm specified
ind = find(strcmp('arm',varargin));
if ind
    arm = varargin{ind+1};
else
    % If not specified, just use arm0
    arm = 0;
end

col_locations = table;

line = fgets(logfile);
while (ischar(line))
    % Get arm number and first word
    matches = regexp(line, 'arm([0123]) (\w*)', 'tokens');
    %is this our arm?
    if ~isempty(matches) && (str2num(matches{1}{1}) == arm)

        % Handle eef pose line
        if strcmp(matches{1}{2}, 'commanded_eef_pose')
            eeftokens = regexp(line, 'commanded_eef_pose=(\d*) \.\.\. (\d*)', 'tokens');
            if size(eeftokens{1},2) ~= 2
                disp('EEF token parse error:');
                disp(line);
                return;
            end
            col_locations.commanded_eef_pose = str2num(eeftokens{1}{1}):str2num(eeftokens{1}{2});
            
        % Handle all joint lines:   
        elseif any(strcmp(matches{1}{2}, {'elbow', 'forearm', 'pit_a', 'pit_b', 'pit_c', 'should_p', ...
                'should_r', 'spher_b', 'spher_r', 'rotate', 'transl', 'proxim', 'distal', 'jaw_a', 'jaw_b'}))
            jointcmdtoken = regexp(line, 'cmd=(\d*)', 'tokens');
            if size(jointcmdtoken{1},2) ~= 1
                disp('Joint parse error for joint: ', matches{1}{2});
                disp(line);
                return;
            end
            col_locations.(matches{1}{2}) = str2num(jointcmdtoken{1}{1});
        end
    elseif ~any(regexp(line, '[a-z]'))
        % Stop parsing if we have a line with now lowercase letters (means
        % we're done with the header)
        break;
    end
    line = fgets(logfile);
end

if size(col_locations,2) ~= 16
    disp('Parse error, did not find expected 12 columns. Found:');
    col_locations
    return
end

col_locations

%%

data = import_log(logname);
ws = [-1,1,-1,1,-1,1];
t_tmp = data(:,2)-data(1,2);

%Columns we want:
qcols = [col_locations.should_p, col_locations.should_r, col_locations.elbow, ...
    col_locations.forearm, col_locations.spher_b, col_locations.spher_r,      ...
    col_locations.pit_a, col_locations.pit_b, col_locations.pit_c,            ...
    col_locations.transl, col_locations.rotate];
eef_cols = col_locations.commanded_eef_pose;
wrist_cols = [col_locations.proxim, col_locations.distal, col_locations.jaw_a, col_locations.jaw_b];


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
    q = [q(:,1:6),sum(q(:,7:9),2),q(:,10:end)]; % Sum pitch joint into one joint
end

wrist = data(good_inds,wrist_cols);
% Calculate jaw angle
wrist = [wrist(:,1), wrist(:,2), wrist(:,4) - wrist(:,3)];
% If jaws try to go less than 0, set them to 0
wrist(wrist(:,3) < 0, 3) = 0;



eef = data(good_inds,eef_cols);
t = data(good_inds,2)-data(1,2);

ti = 0:interp_dt:max(t);

if any(strcmp('spline',varargin))
    eef = interp1(t,eef,ti,'spline');
    q = interp1(t,q,ti,'spline');
    wrist = interp1(t,wrist,ti,'spline');
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
    wrist_filt = filtfilt(b, a, wrist);
    q = qs;
    wrist = wrist_filt;

end


