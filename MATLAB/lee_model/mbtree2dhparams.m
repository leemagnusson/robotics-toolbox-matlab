function dh_params = mbtree2dhparams(mbtree)
% MBTREE2DHPARAMS Generates a DH parameter table from a mbtree struct.
dh_params = [];
% Create sorted list of the links in parent/child descending order
root_index = 0;
for c=1:length(mbtree.rigidbodies)
    if isempty(mbtree.rigidbodies(c).parent)
        if root_index == 0
            root_index = c;
        else
            warning(['Additional root body found: ' ...
                mbtree.rigidbodies(c).name ...
                ' Root body assumed to be: ' ...
                mbtree.rigidbodies(root_index).name]);
        end
    end
end
if root_index == 0
    error('No root body found for this tree');
end
body_list = [];
child_index = root_index;
while(~isempty(child_index))
    body_list(end + 1) = child_index;
    child_index = mbtree.rigidbodies(child_index).children;
    if length(child_index) > 1
        child_index = child_index(1);
        warning([ 'Multiple children for: ' ...
            mbtree.rigidbodies(body_list(end)).name ...
            ' Selecting only: ' ...
            mbtree.rigidbodies(child_index).name]);
    end
end
% Now work through each rigid body, combining fixed transformations up to
% the first DOF and then express this DOF as if it were in DH parameters.

% d      - offset along previous z to the common normal
% th     - angle about previous z, from old x to new x
% r      - length of the common normal. Assuming a revolute joint, this is
%          the radius about previous z.
% a      - angle about common normal, from old z axis to new z axis
%
%
% Then the DH homogenous transform is of the form:
%
% base_T_tip =
%              [ cos(th), -sin(th)*cos(a),  sin(th)*sin(a),  r*cos(th) ]
%              [ sin(th),  cos(th)*cos(a), -cos(th)*sin(a),  r*sin(th) ]
%              [       0,          sin(a),          cos(a),          d ]
%              [       0,               0,               0,          1 ]
%
% where all parameters (th, a, r, d) are for the "tip"-th frame.
transform = eye(4);
for b=body_list(2:end) % No need to find DH params for root body
    body = mbtree.rigidbodies(b);
    transform(1:3,4) = body.origin;
    for d=body.dofs
        dof = mbtree.transforms(d);
        transform = transform * dof.transform;
        if ~dof.locked
            if isAxisZ(dof.axis)
                if isDhCompliantTransform(transform)
                    dh = extractDhParameters(transform);
                    transform = eye(4);
                else
                    warning([
                    'Joint transform not compliant ', ...
                    'with DH convention. Attempting to correct: ', ...
                    dof.name]);
                    % Attempt to correct a non-compliant transform
                    rotz = zRotation(solveForDhCorrection(transform));
                    if ~isDhCompliantTransform(transform * rotz)
                        error([
                            'Unable to correct non-compliant ', ...
                            'transform: ', dof.name]);
                    end
                    dh = extractDhParameters(transform * rotz);
                    % Rotate the frame back to pre-correction so as to not
                    % disturb the meaning of the following frames.
                    transform = rotz';
                end
                if isempty(dh_params)
                    dh_params = dh;
                else
                    dh_params(end+1) = dh;
                end
            else
                error([
                    'Unable to handle joint-axes that are not', ...
                    ' aligned with the z-axis. Failing on transform: ', ...
                    dof.name]);
            end
        end
    end
end

function angle = solveForDhCorrection(transform)
% cos(t)*T31+sin(t)*T32 = 0
% cos(t)*T31 = -sin(t)*T32
% sin(t) / cos(t) = -T31 / T32
angle = atan2(-transform(3,1), transform(3,2));

function transform = zRotation(angle)
cz = cos(angle);
sz = sin(angle);
transform = [ ...
    cz, -sz, 0, 0; ...
    sz, cz, 0, 0; ...
    0, 0, 1, 0; ...
    0, 0, 0, 1; ];

function tf = isAxisZ(axis)
axis = axis / norm(axis);
tf = ...
    (abs(axis(1)) < eps) && ...
    (abs(axis(2)) < eps) && ...
    ((1-abs(axis(3))) < eps);

function tf = isDhCompliantTransform(transform)
tf = abs(transform(3,1)) < 1e-5;

function dh = extractDhParameters(transform)
% base_T_tip =
%              [ cos(th), -sin(th)*cos(a),  sin(th)*sin(a),  r*cos(th) ]
%              [ sin(th),  cos(th)*cos(a), -cos(th)*sin(a),  r*sin(th) ]
%              [       0,          sin(a),          cos(a),          d ]
%              [       0,               0,               0,          1 ]
dh.d = transform(3,4);
dh.th = atan2(transform(1,2), transform(1,1));
dh.a = atan2(transform(3,2), transform(3,3));
dh.r = sqrt(transform(1,4)^2 + transform(2,4)^2);
