%------- function R = rotr(n,phi); -----------------
% computes the rotation matrix R for rotation about axis n with phi radians
%-----------------------------------------------------------
function R = rotr(n,phi)
if length(n) == 3 && ismember(1,size(n))
    n = n ./ norm(n);
    s = phi*[0 -n(3) n(2);n(3) 0 -n(1);-n(2) n(1) 0];
    R = expm(s);
else
    error('Wrong vector size');
end
return;

