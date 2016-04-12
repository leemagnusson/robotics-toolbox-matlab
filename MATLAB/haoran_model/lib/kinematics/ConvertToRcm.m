function q_rcm = ConvertToRcm(q,coupling_matrix)
% this function converts active joint angles 11 by 1 to coupled joint
% angles 13 by 1, coupling_matrix is 13 by 11 matrix
% q_rcm = ConvertToRcm(q,coupling_matrix)

q_rcm = coupling_matrix*q;
end