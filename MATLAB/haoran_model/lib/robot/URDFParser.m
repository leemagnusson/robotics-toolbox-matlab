function [ urdf_in] = URDFParser( urdf_file_name )
% urdf_input = URDFParser('urdf_file_name.URDF');

urdf_struct = parseXML(urdf_file_name);
link_num = 0;
joint_num = 0;
urdf_in = struct('joints',[],'links',[],'jseq',[]);
for index = 1:length(urdf_struct.Children)
    if strcmp(urdf_struct.Children(index).Name, 'link')
        link_num = link_num + 1;
        urdf_in.links{link_num} = struct('name',urdf_struct.Children(index).Attributes.Value,...
            'inertial',[],...
            'visual',[],...
            'collision',[]);
        for index2 = 1:length(urdf_struct.Children(index).Children)
            
            if strcmp(urdf_struct.Children(index).Children(index2).Name, 'inertial')
                urdf_in.links{link_num}.inertial = struct('origin',[],'mass',[],'inertia',[]);
                for index3 = 1:length(urdf_struct.Children(index).Children(index2).Children)
                    if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'mass')
                        urdf_in.links{link_num}.inertial.mass = struct('value',str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes.Value));
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'origin')
                        urdf_in.links{link_num}.inertial.origin = struct('rpy',[],'xyz',[]);
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Attributes)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'rpy')
                                urdf_in.links{link_num}.inertial.origin.rpy = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'xyz')
                                urdf_in.links{link_num}.inertial.origin.xyz = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            end
                        end
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'inertia')
                        urdf_in.links{link_num}.inertial.inertia = struct('ixx',[],'ixy',[],'ixz',[],'iyy',[],'iyz',[],'izz',[]);
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Attributes)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'ixx')
                                urdf_in.links{link_num}.inertial.inertia.ixx = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'ixy')
                                urdf_in.links{link_num}.inertial.inertia.ixy = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'ixz')
                                urdf_in.links{link_num}.inertial.inertia.ixz = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'iyy')
                                urdf_in.links{link_num}.inertial.inertia.iyy = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'iyz')
                                urdf_in.links{link_num}.inertial.inertia.iyz = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'izz')
                                urdf_in.links{link_num}.inertial.inertia.izz = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            end
                        end
                    end
                end
                
            elseif strcmp(urdf_struct.Children(index).Children(index2).Name, 'visual')
                urdf_in.links{link_num}.visual = struct('origin',[],'geometry',[],'material',[]);
                for index3 = 1:length(urdf_struct.Children(index).Children(index2).Children)
                    if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'origin')
                        urdf_in.links{link_num}.visual.origin = struct('rpy',[],'xyz',[]);
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Attributes)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'rpy')
                                urdf_in.links{link_num}.visual.origin.rpy = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'xyz')
                                urdf_in.links{link_num}.visual.origin.xyz = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            end
                        end
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'geometry')
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Children)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Children(index4).Name, 'mesh')
                                urdf_in.links{link_num}.visual.geometry = struct('mesh',struct('filename',urdf_struct.Children(index).Children(index2).Children(index3).Children(index4).Attributes.Value));
                            end
                        end
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'material')
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Children)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Children(index4).Name, 'color')
                                urdf_in.links{link_num}.visual.material = struct('color',struct('rgba',str2num(urdf_struct.Children(index).Children(index2).Children(index3).Children(index4).Attributes.Value)));
                            end
                        end
                        
                    end
                end
                elseif strcmp(urdf_struct.Children(index).Children(index2).Name, 'collision')
                urdf_in.links{link_num}.collision = struct('origin',[],'geometry',[]);
                for index3 = 1:length(urdf_struct.Children(index).Children(index2).Children)
                    if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'origin')
                        urdf_in.links{link_num}.collision.origin = struct('rpy',[],'xyz',[]);
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Attributes)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'rpy')
                                urdf_in.links{link_num}.collision.origin.rpy = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Name, 'xyz')
                                urdf_in.links{link_num}.collision.origin.xyz = str2num(urdf_struct.Children(index).Children(index2).Children(index3).Attributes(index4).Value);
                            end
                        end
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Name,'geometry')
                        for index4 = 1:length(urdf_struct.Children(index).Children(index2).Children(index3).Children)
                            if strcmp(urdf_struct.Children(index).Children(index2).Children(index3).Children(index4).Name, 'mesh')
                                urdf_in.links{link_num}.collision.geometry = struct('mesh',struct('filename',urdf_struct.Children(index).Children(index2).Children(index3).Children(index4).Attributes.Value));
                            end
                        end
                    end
                end
            end
        end
    elseif strcmp(urdf_struct.Children(index).Name, 'joint')
        joint_num = joint_num + 1;
        urdf_in.joints{joint_num} = struct('name',urdf_struct.Children(index).Attributes(1).Value,...
            'type',urdf_struct.Children(index).Attributes(2).Value,...
            'origin',[],...
            'parent',[],...
            'child',[],...
            'axis',[],...
            'limit',[]);
        if ~strcmp(urdf_struct.Children(index).Attributes(2).Value,'fixed')
            urdf_in.jseq = [urdf_in.jseq joint_num];
        end
        for index2 = 1:length(urdf_struct.Children(index).Children)
            if strcmp(urdf_struct.Children(index).Children(index2).Name, 'origin')
                urdf_in.joints{joint_num}.origin = struct('rpy',[],'xyz',[]);
                for index3 = 1:length(urdf_struct.Children(index).Children(index2).Attributes)
                    if strcmp(urdf_struct.Children(index).Children(index2).Attributes(index3).Name, 'rpy')
                        urdf_in.joints{joint_num}.origin.rpy = str2num(urdf_struct.Children(index).Children(index2).Attributes(index3).Value);
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Attributes(index3).Name, 'xyz')
                        urdf_in.joints{joint_num}.origin.xyz = str2num(urdf_struct.Children(index).Children(index2).Attributes(index3).Value);
                    end
                end
            elseif strcmp(urdf_struct.Children(index).Children(index2).Name, 'parent')
                urdf_in.joints{joint_num}.parent = struct('link',urdf_struct.Children(index).Children(index2).Attributes.Value);
            elseif strcmp(urdf_struct.Children(index).Children(index2).Name, 'child')
                urdf_in.joints{joint_num}.child = struct('link',urdf_struct.Children(index).Children(index2).Attributes.Value);
            elseif strcmp(urdf_struct.Children(index).Children(index2).Name, 'axis')
                urdf_in.joints{joint_num}.axis = struct('xyz',str2num(urdf_struct.Children(index).Children(index2).Attributes.Value));
            elseif strcmp(urdf_struct.Children(index).Children(index2).Name, 'limit')
                urdf_in.joints{joint_num}.limit = struct('effort',[],'lower',[],'upper',[],'velocity',[]);
                for index3 = 1:length(urdf_struct.Children(index).Children(index2).Attributes)
                    if strcmp(urdf_struct.Children(index).Children(index2).Attributes(index3).Name, 'effort')
                        urdf_in.joints{joint_num}.limit.effort = str2num(urdf_struct.Children(index).Children(index2).Attributes(index3).Value);
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Attributes(index3).Name, 'lower')
                        urdf_in.joints{joint_num}.limit.lower = str2num(urdf_struct.Children(index).Children(index2).Attributes(index3).Value);
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Attributes(index3).Name, 'upper')
                        urdf_in.joints{joint_num}.limit.upper = str2num(urdf_struct.Children(index).Children(index2).Attributes(index3).Value);
                    elseif strcmp(urdf_struct.Children(index).Children(index2).Attributes(index3).Name, 'velocity')
                        urdf_in.joints{joint_num}.limit.velocity = str2num(urdf_struct.Children(index).Children(index2).Attributes(index3).Value);
                    end
                end
            end
        end
    end
end
end


function theStruct = parseXML(filename)
% PARSEXML Convert XML file to a MATLAB structure.
try
    tree = xmlread(filename);
catch
    error('Failed to read XML file %s.',filename);
end

% Recurse over child nodes. This could run into problems
% with very deeply nested trees.
try
    theStruct = parseChildNodes(tree);
catch
    error('Unable to parse XML file %s.',filename);
end
end

% ----- Local function PARSECHILDNODES -----
function children = parseChildNodes(theNode)
% Recurse over node children.
children = [];
if theNode.hasChildNodes
    childNodes = theNode.getChildNodes;
    numChildNodes = childNodes.getLength;
    allocCell = cell(1, numChildNodes);
    
    children = struct(             ...
        'Name', allocCell, 'Attributes', allocCell,    ...
        'Data', allocCell, 'Children', allocCell);
    
    for count = 1:numChildNodes
        theChild = childNodes.item(count-1);
        children(count) = makeStructFromNode(theChild);
    end
end
end
% ----- Local function MAKESTRUCTFROMNODE -----
function nodeStruct = makeStructFromNode(theNode)
% Create structure of node info.

nodeStruct = struct(                        ...
    'Name', char(theNode.getNodeName),       ...
    'Attributes', parseAttributes(theNode),  ...
    'Data', '',                              ...
    'Children', parseChildNodes(theNode));

if any(strcmp(methods(theNode), 'getData'))
    nodeStruct.Data = char(theNode.getData);
else
    nodeStruct.Data = '';
end
end

% ----- Local function PARSEATTRIBUTES -----
function attributes = parseAttributes(theNode)
% Create attributes structure.

attributes = [];
if theNode.hasAttributes
    theAttributes = theNode.getAttributes;
    numAttributes = theAttributes.getLength;
    allocCell = cell(1, numAttributes);
    attributes = struct('Name', allocCell, 'Value', ...
        allocCell);
    
    for count = 1:numAttributes
        attrib = theAttributes.item(count-1);
        attributes(count).Name = char(attrib.getName);
        attributes(count).Value = char(attrib.getValue);
    end
end
end

