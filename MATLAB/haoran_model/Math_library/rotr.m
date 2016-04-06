%------- function r = rotr(n,phi); -----------------
% computes the rotation matrix for rotation about axis n phi radians
%-----------------------------------------------------------
function r = rotr(n,phi)
if length(n) == 3 && ismember(1,size(n))
    n = n ./ norm(n);
    s = phi*[0 -n(3) n(2);n(3) 0 -n(1);-n(2) n(1) 0];
    r = expm(s);
else
    error('Wrong vector size');
end
return;

