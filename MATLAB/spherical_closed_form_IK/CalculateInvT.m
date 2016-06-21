function [ Tout ] = CalculateInvT( Tin )
%Calculate the inverse of a symbolic transformation matrix
%Input: 4x4 transformation matrix
if(abs(cond(Tin(1:3,1:3))-1) > eps(1e12))
    str = sprintf(' %f ', Tin);
    error('Transformation is not invertible, %s', str);
else
    Rinv = inv(Tin(1:3,1:3));
    Tout = [Rinv -Rinv*Tin(1:3,4) ; zeros(1,3) 1];
end
end
