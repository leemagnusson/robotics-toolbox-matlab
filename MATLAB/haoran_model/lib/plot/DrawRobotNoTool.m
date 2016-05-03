function DrawRobotNoTool(frames,vertex_arm_origin,arm_color,plot_frame,axis_length)
% this function draws the arm without tool drive

load('/../../data/index_joints.mat')
if nargin > 1
    for index_arm = [1:index_pitch_c index_rcm]
        rotation = frames(1:3,1:3,index_arm);
        d = frames(1:3,4,index_arm);
        if isempty(vertex_arm_origin{1,index_arm}) == 0
            vertex_arm_transformed = transformSTL(vertex_arm_origin(:,index_arm),rotation,d);
            rgba = arm_color(:,index_arm);
            PlotStl(vertex_arm_transformed,rgba)
            hold on
            
        end
        if nargin > 3
            if ismember(index_arm,plot_frame)
                DrawCoordinateSystem([axis_length axis_length axis_length],rotation,d,'rgb',num2str(index_arm))
                hold on
            end
        end
    end
else
    msgbox('wrong input number!')
end