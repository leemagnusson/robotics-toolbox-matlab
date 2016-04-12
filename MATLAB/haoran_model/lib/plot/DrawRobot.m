%% Draw one arm of the robot
% DrawRobot(Frames,vertex_arm_origin,arm_color,plot_frame,axis_length)
% This function draws one arm of the robot
% needs at least 3 arguments
% 4th and 5th argument are frame names and axis length
%%
function DrawRobot(frames_robot,vertex_arm_origin,arm_color,plot_frame,axis_length)
if nargin > 1
    for i = 1:length(frames_robot)
        rotation = frames_robot(1:3,1:3,i);
        translation = frames_robot(1:3,4,i);
        if isempty(vertex_arm_origin{1,i}) == 0
            vertex_arm_transformed = transformSTL(vertex_arm_origin(:,i),rotation,translation);
            rgba = arm_color(:,i);
            PlotStl(vertex_arm_transformed,rgba)
            hold on
        end
        if nargin > 3
            if ismember(i,plot_frame)
                DrawCoordinateSystem([axis_length axis_length axis_length],rotation,translation,'rgb',num2str(i))
                hold on
            end
        end
    end
else
    msgbox('wrong input number!')
end