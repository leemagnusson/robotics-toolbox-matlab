function r_cross = Cross_Product_Matrix(r)
% cross product matrix
% this function converts the r cross to a matrix format
% r_cross = [r X] is a 3 by 3 matrix that satisfy the follwing:
% r_cross * b = r X b. where both r and b are 3 by 1 vectors.
r_cross=[0 -r(3) r(2);...
         r(3) 0 -r(1);...
         -r(2) r(1) 0];
end