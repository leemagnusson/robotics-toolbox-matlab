function Arm_color = get_arm_color(Arm_class)
% this function gets the color of each arm
for i = 1 : length(Arm_class)
    Arm_color(:,i) = Arm_class(i).color';
end
end