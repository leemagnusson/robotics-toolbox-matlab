function q_dot = jinv_control_fun(r,x_dot,J)

B = .0001*diag([ones(1,5),.00001*ones(1,r.n-5)]);
    % direct psuedo inverse
%    q_dot(i,:) = (pinv(J)*x_dot(i,:)')';
    
    % damped least squares 
q_dot = (inv(J'*J+B)*J'*x_dot')';

    

