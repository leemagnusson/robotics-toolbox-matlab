function Draw_Robot_Arm(Arm_Kinematics,VertexData_origin,Frames)
if nargin > 1
    for i = 1:18
        T = Arm_Kinematics(i).Tran_matrix;
        R = T(1:3,1:3);
        d = T(1:3,4);
        if isempty(VertexData_origin{1,i}) == 0
            VertexData_tran = transformSTL(VertexData_origin(:,i),R,d);
            rgba = Arm_Kinematics(i).color;
            plotSTL(VertexData_tran,rgba)
            hold on
            
        end
        if nargin > 2
            if ismember(i,Frames)
                draw_coordinate_system([0.07 0.07 0.07],R,d,'rgb',num2str(i))
                hold on
            end
        end
    end
else
    msgbox('wrong input number!')
end