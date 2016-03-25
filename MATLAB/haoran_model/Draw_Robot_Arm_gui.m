function Draw_Robot_Arm_gui(Arm_Kinematics,VertexData_origin,handles,Frames)
if nargin > 2
    for i = 1:18
        T = Arm_Kinematics(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics(i).color;
            plotSTL_gui(VertexData_tran,rgba,handles)
            hold on
            
        end
        if nargin > 3
            if ismember(i,Frames)
                draw_coordinate_system([0.07 0.07 0.07],R,d,'rgb',num2str(i))
                hold on
            end
        end
    end
else
    msgbox('wrong input number!')
end