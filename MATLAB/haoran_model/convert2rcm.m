function q_rcm = convert2rcm(q)
load('coupling_matrix.mat')
q_rcm = A*q;
end