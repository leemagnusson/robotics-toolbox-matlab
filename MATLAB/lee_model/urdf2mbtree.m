function mb_tree = urdf2mbtree(filename)
% URDF2MBTREE Dynamics info from URDF converted to multi-body tree struct

try
    xml_dom = xmlread(filename);
catch
    error('Failed to read URDF: %s.', filename);
end

% Create a new multi-body tree
mb_tree = newMultiBodyTree();

% Get pointers to important URDF elements
links = xml_dom.getElementsByTagName('link');
joints = xml_dom.getElementsByTagName('joint');

% Populate all rigid bodies and give them names
for c = 1:links.getLength()
    body_name = parseValue(links.item(c-1), 'name');
    if ~isempty(body_name)
        % Create a rigid body
        if isempty(mb_tree.rigidbodies)
            mb_tree.rigidbodies = newRigidBody(body_name);
        else
            mb_tree.rigidbodies(c) = newRigidBody(body_name);
        end
        % Map its index to a name
        mb_tree.map_body.(body_name) = c;
        % Update the inertial properties
        inertial = getFirstElement(links.item(c-1), 'inertial');
        if ~isempty(inertial)
            mb_tree.rigidbodies(c).inertia = newRigidBodyInertia(inertial);
        else
            warning(['Setting default inertial properties for: ' body_name]);
        end
    else
        warning('Skipping bad link tag.');
    end
end

% Walk over all joints and add degrees-of-freedom
for c = 1:joints.getLength()
    joint_name = parseValue(joints.item(c-1), 'name');
    if ~isempty(joint_name)
        child_name = parseValue(getFirstElement(joints.item(c-1), 'child'), 'link');
        if (isfield(mb_tree.map_body, child_name))
            parent_name = parseValue(getFirstElement(joints.item(c-1), 'parent'), 'link');
            if (isfield(mb_tree.map_body, parent_name))
                rb_index = mb_tree.map_body.(child_name);
                if (isempty(mb_tree.rigidbodies(rb_index).parent))
                    p_index = mb_tree.map_body.(parent_name);
                    % Set the parent rigid body
                    mb_tree.rigidbodies(rb_index).parent = p_index;
                    % Set the child for the parent
                    mb_tree.rigidbodies(p_index).children(end + 1) = rb_index;
                    % Determine the joint type
                    dof_type = 'translation';
                    is_fixed = false;
                    joint_type = parseValue(joints.item(c-1), 'type');
                    switch(joint_type)
                        case {'revolute', 'continuous'}
                            dof_type = 'rotation';
                        case 'prismatic'
                            dof_type = 'translation';
                        case 'fixed'
                            is_fixed = true;
                        otherwise
                            warning('Setting joint ' + joint_name + ' as a fixed joint.')
                    end
                    % Setup an axis for the DOF
                    axis_element = getFirstElement(joints.item(c-1), 'axis');
                    if isempty(axis_element)
                        axis = [1; 0; 0];
                    else
                        axis = parseTriplet(axis_element, 'xyz');
                    end
                    % Set the origin of the rigid body with respect to the parent
                    xyz = parseTriplet(getFirstElement(joints.item(c-1), 'origin'), 'xyz');
                    mb_tree.rigidbodies(rb_index).origin = xyz;
                    % Setup the fixed rotations
                    rpy = parseTriplet(getFirstElement(joints.item(c-1), 'origin'), 'rpy');
                    fixed_dof = newDegreeOfFreedom(['base_' joint_name], 'rotation', [1;0;0], 0.0, true);
                    fixed_dof.transform(1:3,1:3) = newRotationFromRPY(rpy);
                    if isempty(mb_tree.transforms)
                        mb_tree.transforms = fixed_dof;
                    else
                        mb_tree.transforms(end + 1) = fixed_dof;
                    end
                    mb_tree.rigidbodies(rb_index).dofs(end + 1) = length(mb_tree.transforms);
                    % If joint is not fixed, setup the state for it
                    if (~is_fixed)
                        mb_tree.transforms(end + 1) = newDegreeOfFreedom(joint_name, dof_type, axis, 0.0, is_fixed);
                        mb_tree.map_dofs.(joint_name) = length(mb_tree.transforms);
                        mb_tree.rigidbodies(rb_index).dofs(end + 1) = length(mb_tree.transforms);
                    end
                else
                    warning(['Ignoring ' joint_name ' because rigid body ' child_name ' already has parent body ' mb_tree.rigidbodies(rb_index).parent.name]);
                end
            else
                warning(['Ignoring ' joint_name ' because it is requesting an unknown parent link: ' parent_name]);
            end
        else
            warning(['Ignoring ' joint_name ' because it is requesting an unknown child link: ' child_name]);
        end
    else
        warning('Skipping bad joint tag.');
    end
end

%---------------- Get First Element
function element = getFirstElement(tree, tagname)
element = [];
elements = tree.getElementsByTagName(tagname);
if ~isempty(elements)
    element = elements.item(0);
end

%---------------- Parse Attribute Value
function value = parseValue(element, attribute)
value = [];
if ~isempty(element)
    string = element.getAttribute(attribute);
    if ~isempty(string)
        char_array = string.toCharArray()';
        value = str2num(char_array);
        if isempty(value)
            value = char_array;
        end
    end
end

%---------------- Parse Attribute Vector3
function value = parseTriplet(element, attribute)
value = zeros(3,1);
if ~isempty(element)
    string = element.getAttribute(attribute);
    if ~isempty(string)
        string = strsplit(string.toCharArray()', ' ');
        if length(string) == 3
            value = [ ...
                str2double(string(1)); ...
                str2double(string(2)); ...
                str2double(string(3)) ];
        end
    end
end

%---------------- Parse URDF Inertial component
function inertia_vector = parseInertiaVector(element)
inertia_vector = [ ...
    parseValue(element, 'ixx'), ... % Ixx
    parseValue(element, 'iyy'), ... % Iyy
    parseValue(element, 'izz'), ... % Izz
    parseValue(element, 'ixy'), ... % Ixy
    parseValue(element, 'ixz'), ... % Ixz
    parseValue(element, 'iyz')];    % Iyz

%---------------- Create Multi-Body Struct
function multibody = newMultiBodyTree()
multibody = struct();
multibody.rigidbodies = [];
multibody.transforms = [];
multibody.map_body = struct();
multibody.map_dofs = struct();

%---------------- Create Rigid Body Struct
function rigidbody = newRigidBody(name)
rigidbody = struct();
rigidbody.name = name;
rigidbody.parent = [];
rigidbody.children = [];
rigidbody.origin = zeros(3,1);
rigidbody.inertia.mass = 0;
rigidbody.inertia.center_of_mass = zeros(3,1);
rigidbody.inertia.tensor = zeros(3);
rigidbody.dofs = [];

%---------------- Create Inertia Struct
function inertia = newRigidBodyInertia(element)
origin_xyz = parseTriplet(getFirstElement(element, 'origin'),'xyz');
origin_rpy = parseTriplet(getFirstElement(element, 'origin'),'rpy');
mass = parseValue(getFirstElement(element, 'mass'), 'value');
m = parseInertiaVector(getFirstElement(element, 'inertia'));

inertia.mass = mass;
inertia.center_of_mass = origin_xyz;
% TODO: Need to handle possible rotations of the inertial frame
inertia.tensor = [ ...
    m(1), m(4), m(5); ...
    m(4), m(2), m(6); ...
    m(5), m(6), m(3); ];

%---------------- Create Degree of Freedom struct
function dof = newDegreeOfFreedom(name, type, axis, state, locked)
dof.name = name;
dof.type = type;
dof.axis = axis;
dof.state = state;
dof.locked = locked;
dof.transform = eye(4);

%---------------- Create Rotation Matrix from RPY
function rot = newRotationFromRPY(rpy)
cx = cos(rpy(1));
sx = sin(rpy(1));
cy = cos(rpy(2));
sy = sin(rpy(2));
cz = cos(rpy(3));
sz = sin(rpy(3));
rx = [ 1, 0, 0; 0, cx, -sx; 0, sx, cx; ];
ry = [ cy, 0, sy; 0, 1, 0; -sy, 0, cy; ];
rz = [ cz, -sz, 0; sz, cz, 0; 0, 0, 1; ];
rot = rz*ry*rx;
