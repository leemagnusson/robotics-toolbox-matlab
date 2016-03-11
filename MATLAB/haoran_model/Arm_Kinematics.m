%% Arm kinematics class definition
% Arm Kinematics is a class we generate to take the input from:
% URDF file: '****.URDF'
% joint input: q 9X1 vector
% calculates the kinematics and store the kinematics and dynamics related
% parameters in a class.
% written by Haoran Yu
% 3/9/2016
%%

classdef Arm_Kinematics
    properties
        name
        Tran_matrix
        stl_name
        mass
        center_of_mass
        inertia_matrix
        color
        parent
    end
    methods
        function Arm_Kinematics_out = Arm_Kinematics(link_input,joint_input,q)
            load('parent_number.mat');
            if nargin ~=0
            Arm_Kinematics_out(length(link_input)) = Arm_Kinematics;
            Arm_Kinematics_out(1).Tran_matrix = eye(4);
            for i = 1:length(link_input)
                Arm_Kinematics_out(i).parent = parent_number(1,i);
                joint_number = parent_number(2,i);
                if isfield(link_input{i},'visual')
                Arm_Kinematics_out(i).color = link_input{i}.visual.material.color.rgba;
                else
                    Arm_Kinematics_out(i).color = [1 1 1 1];
                end
                Arm_Kinematics_out(i).name = link_input{i}.name;
                if i>1
                
                d = joint_input{joint_number}.origin.xyz;
                euler = joint_input{joint_number}.origin.rpy;
                
%                 if i>12 || i==2
                if i>12
                    joint_angle = 0;
                    rot_axis = [0;0;1];
                else
                    joint_angle = q(i-1);
                    rot_axis = joint_input{i-1}.axis.xyz;
                end
                rot_matrix = RotationMatrix_rad(euler(3),[0;0;1])*...
                             RotationMatrix_rad(euler(2),[0;1;0])*...
                             RotationMatrix_rad(euler(1),[1;0;0])*...
                             RotationMatrix_rad(joint_angle,rot_axis);
                T = [rot_matrix d'; 0 0 0 1];
                Arm_Kinematics_out(i).Tran_matrix = T;
                end
                Arm_Kinematics_out(i).stl_name = strcat(link_input{i}.name,'.stl');
                if isfield(link_input{i},'inertial')
                Arm_Kinematics_out(i).mass = link_input{i}.inertial.mass;
                Arm_Kinematics_out(i).center_of_mass = link_input{i}.inertial.origin;
                Arm_Kinematics_out(i).inertia_matrix = [link_input{i}.inertial.inertia.ixx link_input{i}.inertial.inertia.ixy link_input{i}.inertial.inertia.ixz;...
                                                        link_input{i}.inertial.inertia.ixy link_input{i}.inertial.inertia.iyy link_input{i}.inertial.inertia.iyz;...
                                                        link_input{i}.inertial.inertia.ixz link_input{i}.inertial.inertia.iyz link_input{i}.inertial.inertia.izz];
                else
                    Arm_Kinematics_out(i).mass = 0;
                    Arm_Kinematics_out(i).center_of_mass = 0;
                    Arm_Kinematics_out(i).inertia_matrix = zeros(3,3);
                end
                
            end
            for i = 2:length(link_input)
                Arm_Kinematics_out(i).Tran_matrix = Arm_Kinematics_out(Arm_Kinematics_out(i).parent).Tran_matrix * Arm_Kinematics_out(i).Tran_matrix;
            end
            end
        end
    end
end
