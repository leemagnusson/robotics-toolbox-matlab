function m = get_motors(varargin)
% read motor file and calculate additional values

m = loadjson('../data/robodrive.json');
mnames = fieldnames(m);
mnames = mnames(2:end);
c = {};

for i = 1:length(mnames)
    s = mnames{i};
    m.(s).km = m.(s).kc*sqrt(m.(s).R);
    m.(s).kv = 1/m.(s).km;
    c{i} = m.(s);
end

if(strcmp('cell',varargin))
    m=c;
end