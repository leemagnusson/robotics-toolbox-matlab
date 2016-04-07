function PlotStl(vertex,rgba,axes_handle)
% Plot stl files
% 
if nargin == 2
    hPlot = patch(vertex{1},vertex{2},vertex{3},zeros(size(vertex{1})));
    set(hPlot,'facec',rgba(1:3),'EdgeColor','none','FaceLighting','flat');
    alpha(hPlot,rgba(4));
    axis image
elseif nargin == 3
    hPlot = patch(vertex{1},vertex{2},vertex{3},zeros(size(vertex{1})),'parent',axes_handle);
    set(hPlot,'facec',rgba(1:3),'EdgeColor','none','FaceLighting','flat');
    alpha(hPlot,rgba(4));
    axis image
else
    error('Wrong input arguments');
end
