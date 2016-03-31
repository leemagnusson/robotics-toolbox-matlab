function q_rcm = convert2rcm(q)
% this function converts active joint angles 11 by 1 to coupled joint
% angles 13 by 1
% by Haoran Yu 3/22/2016
load('coupling_matrix.mat')
q_rcm = A*q;
end