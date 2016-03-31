function r_cross = CPM(r)
% cross product matrix
% this function converts the r cross to a matrix format
% by Haoran Yu 3/22/2016
r_cross=[0 -r(3) r(2); r(3) 0 -r(1);-r(2) r(1) 0];
end