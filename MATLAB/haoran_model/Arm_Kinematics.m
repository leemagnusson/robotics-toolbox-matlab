%% Arm kinematics class definition
% Arm Kinematics is a class we generate to take the input from:
% link_input and joint_input from URDF file: '****.URDF'
% joint input: q 13X1 vector
% calculates the kinematics and store the kinematics and dynamics related
% parameters in a class.
% written by Haoran Yu
% 3/9/2016
%%

classdef Arm_Kinematics
    properties
        name % name of the link and joint pair
        Tran_matrix % transformation matrix of the current link and joint to the world
        stl_name % name of the stl
        mass % mass of the link
        center_of_mass % center of mass of the link
        inertia_matrix % inertia matrix of the link
        color % color of the link
        parent % parent link/joint pair
    end
    methods
        function Arm_Kinematics_out = Arm_Kinematics(link_input,joint_input,q_rcm,base_T)
            load('parent_number.mat');
            if nargin ~=0
                Arm_Kinematics_out(length(link_input)) = Arm_Kinematics;
                
                for i = 1:length(link_input)
                    % set parent and joint number from the mat file
                    Arm_Kinematics_out(i).parent = parent_number(1,i);
                    joint_number = parent_number(2,i);
                    % set the color if link input has 'visual' field
                    if isfield(link_input{i},'visual')
                        Arm_Kinematics_out(i).color = link_input{i}.visual.material.color.rgba;
                    else
                        Arm_Kinematics_out(i).color = [1 1 1 1];
                    end
                    % set the name following the link name
                    Arm_Kinematics_out(i).name = link_input{i}.name;
                    % Define homogeneous transformation
                    if i==1
                        % base link
                        Arm_Kinematics_out(i).Tran_matrix = base_T;
                    else
                        % joint translation and fixed frame roll pitch yaw
                        d = joint_input{joint_number}.origin.xyz;
                        euler = joint_input{joint_number}.origin.rpy;
                        % active joint axis and joint value
                        if i>14
                            joint_value = 0;
                            joint_axis = [0;0;1];
                        else
                            joint_value = q_rcm(joint_number);
                            joint_axis = joint_input{joint_number}.axis.xyz;
                        end
                        % homogeneous transformation for revolute and
                        % prismatic joints
                        if i~=11
                            rot_matrix = RotationMatrix_rad(euler(3),[0;0;1])*...
                                RotationMatrix_rad(euler(2),[0;1;0])*...
                                RotationMatrix_rad(euler(1),[1;0;0])*...
                                RotationMatrix_rad(joint_value,joint_axis);
                            T = [rot_matrix d'; 0 0 0 1];
                        else
                            rot_matrix = RotationMatrix_rad(euler(3),[0;0;1])*...
                                RotationMatrix_rad(euler(2),[0;1;0])*...
                                RotationMatrix_rad(euler(1),[1;0;0]);
                            disp = (joint_axis * joint_value)';
                            T = [rot_matrix d' + rot_matrix * disp; 0 0 0 1];
                        end
                        Arm_Kinematics_out(i).Tran_matrix = T;
                    end
                    % set stl file name
                    Arm_Kinematics_out(i).stl_name = strcat(link_input{i}.name,'.stl');
                    % check and set inertial terms for dynamics
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
                % Transform the homonegeous matrices to world frame
                for i = 2:length(link_input)
                    Arm_Kinematics_out(i).Tran_matrix = Arm_Kinematics_out(Arm_Kinematics_out(i).parent).Tran_matrix * Arm_Kinematics_out(i).Tran_matrix;
                end
            end
        end
    end
end
