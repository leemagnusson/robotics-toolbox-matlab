function Draw_Robot_Arm_no_tool(Frames,VertexData_origin,Arm_color,plot_frame,axis_length)
% this function draws the arm without tool drive
% by Haoran Yu 3/31/2016
load('/../data_store/index_joints.mat')
if nargin > 1
    for i = [1:index_pitch_c index_rcm]
        R = Frames(1:3,1:3,i);
        d = Frames(1:3,4,i);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_color(:,i);
            plotSTL(VertexData_tran,rgba)
            hold on
            
        end
        if nargin > 3
            if ismember(i,plot_frame)
                draw_coordinate_system([axis_length axis_length axis_length],R,d,'rgb',num2str(i))
                hold on
            end
        end
    end
else
    msgbox('wrong input number!')
end