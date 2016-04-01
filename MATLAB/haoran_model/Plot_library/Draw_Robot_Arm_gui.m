%% Draw robot in gui
% same as Draw robot but takes GUI handle to plot on axis
% by Haoran Yu 3/22/2016
%%

function Draw_Robot_Arm_gui(Frames,VertexData_origin,Arm_color,axes_handle,plot_frame,axis_length)
if nargin > 1
    for i = 1:length(Frames)
        R = Frames(1:3,1:3,i);
        d = Frames(1:3,4,i);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_color(:,i);
            plotSTL_gui(VertexData_tran,rgba,axes_handle)
            hold on
            
        end
        if nargin > 4
            if ismember(i,plot_frame)
                draw_coordinate_system([axis_length axis_length axis_length],R,d,'rgb',num2str(i))
                hold on
            end
        end
    end
else
    msgbox('wrong input number!')
end