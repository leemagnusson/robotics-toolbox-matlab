%% Arm kinematics class definition
% Arm Kinematics is a class we generate to take the input from:
% link_input and joint_input from URDF file: '****.URDF'
% modified by Haoran Yu
% name: name of the link and joint pair
% stl_name: name of the stl
% mass: mass of the link
% center_of_mass: center of mass of the link
% inertia_matrix: inertia matrix of the link
% color: color of the link
% parent: parent link/joint pair
% jnt_limit: joint limit
% rpy: roll pitch yaw angles
% xyz: translation from parent
% jnt_axis: active rotation jnt axis
% type: link type
% Arm_Kinematics.calc_FK: calculate forward kinematics and return Frames
% with 4 by 4 by n format.
% 3/31/2016
%%

classdef Arm_Kinematics
    properties
        name % name of the link and joint pair
        stl_name % name of the stl
        mass % mass of the link
        center_of_mass % center of mass of the link
        inertia_matrix % inertia matrix of the link
        color % color of the link
        parent % parent link/joint pair
        jnt_limit % joint limit
        rpy % roll pitch yaw angles
        xyz % translation from parent
        jnt_axis % active rotation jnt axis
        type % link type
    end
    methods
        function Arm_Kinematics_out = Arm_Kinematics(link_input,joint_input)
            % load the parent number and joint number.
            load('/../data_store/parent_number.mat');
            if nargin ~=0
                Arm_Kinematics_out(length(link_input)) = Arm_Kinematics;
                for i = 1:length(link_input)
                    % set parent and joint number from the mat file
                    joint_index = parent_number(2,i);
                    Arm_Kinematics_out(i).parent = parent_number(1,i);
                    % set the color if link input has 'visual' field
                    if isfield(link_input{i},'visual')
                        Arm_Kinematics_out(i).color = link_input{i}.visual.material.color.rgba;
                    else
                        Arm_Kinematics_out(i).color = [1 1 1 1];
                    end
                    % set the name following the link name
                    Arm_Kinematics_out(i).name = link_input{i}.name;
                    % Define joint limits
                    if i==1
                        % base link is initialized differently
                        Arm_Kinematics_out(i).jnt_limit = [[];[]];
                        Arm_Kinematics_out(i).xyz = [0;0;0];
                        Arm_Kinematics_out(i).rpy = [0;0;0];
                        Arm_Kinematics_out(i).jnt_axis = [0;0;1];
                        Arm_Kinematics_out(i).type = 'fixed';
                    else
                        % detect fixed links and set link type andd active
                        % joint axis
                        if strcmp(joint_input{joint_index}.type, 'fixed') || ~isempty(strfind(joint_input{joint_index}.name,'jaw'))
                            Arm_Kinematics_out(i).type = 'fixed';
                            Arm_Kinematics_out(i).jnt_axis = [0;0;1];
                        else
                            Arm_Kinematics_out(i).type = joint_input{joint_index}.type;
                            Arm_Kinematics_out(i).jnt_axis = joint_input{joint_index}.axis.xyz';
                        end
                        
                        % set the xyz translation and rpy euler angles
                        Arm_Kinematics_out(i).xyz = joint_input{joint_index}.origin.xyz';
                        Arm_Kinematics_out(i).rpy = joint_input{joint_index}.origin.rpy';
                        % get joint limits
                        if isfield(joint_input{joint_index},'limit')
                            Arm_Kinematics_out(i).jnt_limit = [joint_input{joint_index}.limit.lower; joint_input{joint_index}.limit.upper];
                        else
                            Arm_Kinematics_out(i).jnt_limit = [[];[]];
                        end
                    end
                    % set stl file name
                    Arm_Kinematics_out(i).stl_name = strcat(link_input{i}.name,'.stl');
                    % check and set inertial terms for dynamics
                    if isfield(link_input{i},'inertial')
                        Arm_Kinematics_out(i).mass = link_input{i}.inertial.mass.value;
                        Arm_Kinematics_out(i).center_of_mass = link_input{i}.inertial.origin.xyz;
                        Arm_Kinematics_out(i).inertia_matrix = [link_input{i}.inertial.inertia.ixx link_input{i}.inertial.inertia.ixy link_input{i}.inertial.inertia.ixz;...
                            link_input{i}.inertial.inertia.ixy link_input{i}.inertial.inertia.iyy link_input{i}.inertial.inertia.iyz;...
                            link_input{i}.inertial.inertia.ixz link_input{i}.inertial.inertia.iyz link_input{i}.inertial.inertia.izz];
                    else
                        Arm_Kinematics_out(i).mass = 0;
                        Arm_Kinematics_out(i).center_of_mass = 0;
                        Arm_Kinematics_out(i).inertia_matrix = zeros(3,3);
                    end
                end
            end
        end
        
        function Frames = calc_FK(Arm_Kinematics_out,q_rcm,base_T)
            load('/../data_store/parent_number.mat');
            if nargin ~=0
                for i = 1:length(Arm_Kinematics_out)
                    % set parent and joint number from the mat file
                    joint_index = parent_number(2,i);
                    % Define homogeneous transformation
                    if i==1
                        % base link
                        Frames(:,:,i) = base_T;
                    else
                        % joint translation and fixed frame roll pitch yaw
                        d = Arm_Kinematics_out(i).xyz;
                        euler = Arm_Kinematics_out(i).rpy;
                        joint_axis = Arm_Kinematics_out(i).jnt_axis;
                        % active joint value
                        if strcmp(Arm_Kinematics_out(i).type, 'fixed')
                            joint_value = 0;
                        else
                            joint_value = q_rcm(joint_index);
                        end
                        % homogeneous transformation for revolute and
                        % prismatic joints
                        if ~strcmp(Arm_Kinematics_out(i).type, 'prismatic')
                            rot_matrix = rotr([0;0;1],euler(3))*...
                                rotr([0;1;0],euler(2))*...
                                rotr([1;0;0],euler(1))*...
                                rotr(joint_axis,joint_value);
                            Frames(:,:,i) = [rot_matrix d; 0 0 0 1];
                        else
                            rot_matrix = rotr([0;0;1],euler(3))*...
                                rotr([0;1;0],euler(2))*...
                                rotr([1;0;0],euler(1));
                            disp = joint_axis * joint_value;
                            Frames(:,:,i) = [rot_matrix d + rot_matrix * disp; 0 0 0 1];
                        end
                    end
                end
                % Transform the homonegeous matrices to world frame
                for i = 2:length(Arm_Kinematics_out)
                    Frames(:,:,i) = Frames(:,:,Arm_Kinematics_out(i).parent) * Frames(:,:,i);
                end
            end
        end
    end
end
