function urdf_preprocess(urdf_in, urdf_out,stl_relative_path)
% Preprocess the urdf to remove "package://", etc.
%   This function is expected to be a little fragile as the urdf changes,
%   but the matlab import expects a certain format
%   stl_relative_path, stl files refered to mesh/ not absolute

if ~exist('stl_relative_path','var')
    stl_relative_path = false;
end



urdf_path = fileparts(urdf_in);
fin = fopen(urdf_in);
sin = char(fread(fin))';
fclose(fin);
if stl_relative_path
    path_out = '';
else
    path_out = strrep([urdf_path, '/../'],'\','/');
end
sout = regexprep(sin,'package://.*?/',path_out);
sout = regexprep(sout,'name="V1pt5_distal_wrist_link">','$0\n\t<mass value="0.001" />');
sout = regexprep(sout,'(<material.*?name=")(">)','$1generic$2');
fout = fopen(urdf_out,'w');
fwrite(fout,sout);
fclose(fout);