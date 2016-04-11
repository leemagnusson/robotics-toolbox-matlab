function [m,r,I] = import_solidworks_mass_properties(filename)

fid = fopen(filename);

s = fgetl(fid);
s = fgetl(fid);
s = fgetl(fid);

if (regexpi(s,'default'))
    disp(s);
    error('should be not in default coordinate system');
end

s = char(fread(fid)');

m_str = regexpi(s,'mass.*?= ([\d.]+)','tokens');
m = str2double(m_str{1})/1000; % g -> kg

r_str = regexpi(s,'center of mass: .*?x = ([-\d.]+).*?y = ([-\d.]+).*?z = ([-\d.]+)', 'tokens'); 
r = str2double(r_str{1})/1000; % mm -> m

I_str = regexpi(s,['moments of inertia.*?center of mass.*?', ...
                   'lxx = ([-\d.]+).*?lxy = ([-\d.]+).*?lxz = ([-\d.]+).*?', ...
                   'lyx = ([-\d.]+).*?lyy = ([-\d.]+).*?lyz = ([-\d.]+).*?', ...
                   'lzx = ([-\d.]+).*?lzy = ([-\d.]+).*?lzz = ([-\d.]+).*?'], ...
                   'tokens');
I = str2double(I_str{1})/1000/1000^2;   % g-mm^2 -> kg-m^2
I = reshape(I,3,3);

fclose(fid);