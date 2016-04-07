function [robot_color_a,robot_color_b] = ChangeLinkColor(robot_color_a,robot_color_b,collision,colliding_arm_index)
% this function checks the collision and colliding arm number and set 
% the color to be red
% [arm_color_a,arm_color_b] = ChangeLinkColor(robot_color_a,robot_color_b,collision,colliding_arm_index)
% arm_color_a and arm_color_b are color for a and b arms in a matrix
% format: 4 by n with each column indicate the rgb and transparency of the
% arm color
% collision and colliding_arm_index: result from function
% FindCollisionTwoRobt 
red_color = [1 0 0 1]';
if collision
    for index = 1:length(robot_color_a)
        if ismember(index,colliding_arm_index(1,:))
            if index == 10
                robot_color_a(:,index) = red_color;
                robot_color_a(:,index+1) = red_color;
            else
                robot_color_a(:,index) = red_color;
            end
        end
        
        if ismember(index,colliding_arm_index(2,:))
            if index == 10
                robot_color_b(:,index) = red_color;
                robot_color_b(:,index+1) = red_color;
            else
                robot_color_b(:,index) = red_color;
            end
        end
    end
end
end