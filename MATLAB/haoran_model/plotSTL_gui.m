function handlePlot=plotSTL_gui(VertexData,rgba,axes_handle)
% PLOTSTL plots surface patches from STL-file
%
% Usage:
%
% handlePlot=plotSTL(VertexData,FVCD)
%
% Input:
%
% VertexData - Matrices with vertices (output from stl2matlab)
% FVCD - FaceVertexColorData or color specification, [r g b] (optional)
% fignr - Figure number of the plot. 1 default
% holdoff_flag - Bolean value (1/0). If 1 then hold off the plot
%                when the plot is done. 1 default
%
% Output:
%
% handlePlot - plothandle
%
% m-file can be downloaded at
% http://www.mathworks.com/matlabcentral/fileexchange/13253-iges-toolbox
%
% written by Per Bergström 2012-01-09
%



    hPlot = patch(VertexData{1},VertexData{2},VertexData{3},zeros(size(VertexData{1})),'parent',axes_handle);
    set(hPlot,'facec',rgba(1:3),'EdgeColor','none','FaceLighting','flat');
%     material metal 
%     set(hPlot,'facec',[0.8 0.4 0.4]);
%     set(hPlot,'facec', rgba(1:3));
    alpha(hPlot,rgba(4));


axis image

sc=0.2;

xl=xlim;
dx=xl(2)-xl(1);
xl(1)=xl(1)-sc*dx;
xl(2)=xl(2)+sc*dx;
xlim(xl);

yl=ylim;
dy=yl(2)-yl(1);
yl(1)=yl(1)-sc*dy;
yl(2)=yl(2)+sc*dy;
ylim(yl);

zl=zlim;
dz=zl(2)-zl(1);
zl(1)=zl(1)-sc*dz;
zl(2)=zl(2)+sc*dz;
zlim(zl);


if nargout>0
    handlePlot=hPlot;
end
