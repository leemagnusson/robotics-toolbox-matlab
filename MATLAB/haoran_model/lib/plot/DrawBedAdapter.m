%% Draw the bed adapter
% This function draws the bed adapter attached to the bed
% DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,bed_adapter_color,axes_handle)
%%
function DrawBedAdapter(frames_bed_adapter,vertex_bed_adapter,bed_adapter_color,axes_handle)
if nargin == 3
    for index_bed_adapter = 1:length(vertex_bed_adapter)
        rotation = frames_bed_adapter(1:3,1:3,index_bed_adapter);
        d = frames_bed_adapter(1:3,4,index_bed_adapter);
        vertex_bed_adapter_transformed = transformSTL(vertex_bed_adapter(:,index_bed_adapter),rotation,d);
        PlotStl(vertex_bed_adapter_transformed,bed_adapter_color);
        hold on
    end
elseif nargin == 4
    for index_bed_adapter = 1:length(vertex_bed_adapter)
        rotation = frames_bed_adapter(1:3,1:3,index_bed_adapter);
        d = frames_bed_adapter(1:3,4,index_bed_adapter);
        vertex_bed_adapter_transformed = transformSTL(vertex_bed_adapter(:,index_bed_adapter),rotation,d);
        PlotStl(vertex_bed_adapter_transformed,bed_adapter_color,axes_handle);
        hold on
    end
else
    msgbox('wrong input number!')
end