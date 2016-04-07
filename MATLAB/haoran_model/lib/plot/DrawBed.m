%% Draw the bed
% This function draws the bed with no bed active joints.
% DrawBed(vertex_bed,bed_color,axes_handle)
%%
function DrawBed(vertex_bed,bed_color,axes_handle)
if nargin == 2
    for i = 1:length(vertex_bed)
        if isempty(vertex_bed{1,i}) == 0
            PlotStl(vertex_bed(:,i),bed_color)
            hold on
        end
    end
elseif nargin == 3
    for i = 1:length(vertex_bed)
        if isempty(vertex_bed{1,i}) == 0
            PlotStl(vertex_bed(:,i),bed_color,axes_handle)
            hold on
        end
    end
else
    msgbox('wrong input number!')
end