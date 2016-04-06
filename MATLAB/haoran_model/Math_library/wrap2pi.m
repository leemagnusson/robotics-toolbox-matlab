function [ angle_out ] = wrap2pi( angle_in )
% this function wrap the angle to -pi to pi
% [angle_out] = wrap2pi(angle_in)
angle_in = mod(angle_in,2*pi);
angle_out = zeros(length(angle_in),1);
for i = 1: length(angle_in)
 if angle_in(i) >= pi
     angle_out(i) = angle_in(i) - 2 * pi;
 else
     angle_out(i) = angle_in(i);
 end
end
end

