function arm_color = GetRobotColor(robot_parameters)
% this function gets the color of each arm
for i = 1 : length(robot_parameters)
    arm_color(:,i) = robot_parameters(i).color_';
end
end