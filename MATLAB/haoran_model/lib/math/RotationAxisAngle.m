%% Rotation matrix axis angle
% calculates the rotation matrix from rotation axis and rotation angle
% rotation = RotationAxisAngle(axis,angle)
% rotation: 3 X 3 rotation matrix
% axis: the rotation axis, 3 by 1 or 1 by 3
% angle: rotation angle in radians.
%%
function rotation = RotationAxisAngle(axis,angle)
if numel(axis)==3 % make sure the axis is 3 by 1 or 1 by 3 vector
    k = reshape(axis ./ norm(axis),3,1);
    rotation = k*(1-cos(angle))*k' + ...
        [cos(angle),      -k(3)*sin(angle),  k(2)*sin(angle);
         k(3)*sin(angle),  cos(angle),      -k(1)*sin(angle);
        -k(2)*sin(angle),  k(1)*sin(angle),  cos(angle)];
else
    error('Wrong vector size');
end
return;