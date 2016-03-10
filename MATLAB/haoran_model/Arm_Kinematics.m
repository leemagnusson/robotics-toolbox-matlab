classdef Arm_Kinematics
    properties
        name
        Tran_matrix
        stl_name
        mass
        center_of_mass
        inertia_matrix
        
    end
    methods
        function Arm_Kinematics_out = Arm_Kinematics(URDF_file,q)
            if nargin ~=0
            urdf_input = URDF(URDF_file);
            link_input = urdf_input.links;
            joint_input = urdf_input.joints;
            Arm_Kinematics_out(length(link_input)) = Arm_Kinematics;
            Arm_Kinematics_out(1).Tran_matrix = eye(4);
            for i = 1:length(link_input)
                Arm_Kinematics_out(i).name = link_input{i}.name;
                if i<length(link_input) && i>1
                d = joint_input{i-1}.origin.xyz;
                euler = joint_input{i-1}.origin.rpy;
                if i>9
                    joint_angle = 0;
                else
                    joint_angle = q(i);
                end
                rot_matrix = RotationMatrix_rad(joint_angle,[0;0;1])*...
                             RotationMatrix_rad(euler(1),[1;0;0])*...
                             RotationMatrix_rad(euler(2),[0;1;0])*...
                             RotationMatrix_rad(euler(3),[0;0;1]);
                T = [rot_matrix d'; 0 0 0 1];
                Arm_Kinematics_out(i).Tran_matrix = Arm_Kinematics_out(i-1).Tran_matrix * T;
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
            end
        end
    end
end
