function [Arm_color_a,Arm_color_b] = Change_link_color(Arm_color_a,Arm_color_b,Collision,collide_index)
% this function checks the collision index and colliding arm number and set
% the color to be red
red_color = [1 0 0 1]';
if Collision
    for index = 1:length(Arm_color_a)
        if ismember(index,collide_index(1,:))
            if index == 10
                Arm_color_a(:,index) = red_color;
                Arm_color_a(:,index+1) = red_color;
            else
                Arm_color_a(:,index) = red_color;
            end
        end
        
        if ismember(index,collide_index(2,:))
            if index == 10
                Arm_color_b(:,index) = red_color;
                Arm_color_b(:,index+1) = red_color;
            else
                Arm_color_b(:,index) = red_color;
            end
        end
    end
end
end