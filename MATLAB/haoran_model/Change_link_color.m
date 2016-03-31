function [Arm_Kinematics_a,Arm_Kinematics_b] = Change_link_color(Arm_Kinematics_a,Arm_Kinematics_b,Collision,collide_index)
if Collision
        for index = 1:length(Arm_Kinematics_a)
            if ismember(index,collide_index(1,:))
                if index == 10
                    Arm_Kinematics_a(index).color = [1 0 0 1];
                    Arm_Kinematics_a(index+1).color = [1 0 0 1];
                else
                    Arm_Kinematics_a(index).color = [1 0 0 1];
                end
            end
            
            if ismember(index,collide_index(2,:))
                if index == 10
                    Arm_Kinematics_b(index).color = [1 0 0 1];
                    Arm_Kinematics_b(index+1).color = [1 0 0 1];
                else
                    Arm_Kinematics_b(index).color = [1 0 0 1];
                end
            end
        end
    end
end