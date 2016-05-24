%% Rotation matrix axis angle
% calculates the rotation matrix from rotation axis and rotation angle
% rotation = RotationAxisAngle(axis,angle)
% rotation: 3 X 3 rotation matrix
% axis: the rotation axis, 3 by 1 or 1 by 3
% angle: rotation angle in radians.
%%
function rotation = RotationAxisAngle(axis,angle)
if length(axis) == 3 && ismember(1,size(axis)) % make sure the axis is 3 by 1 or 1 by 3 vector
    axis = axis ./ norm(axis);
    s = angle*[0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
    rotation = expm(s);
else
    error('Wrong vector size');
end
return;

