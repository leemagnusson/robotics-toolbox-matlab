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
            for i = 1:length(link_input)
                Arm_Kinematics_out(i).name = link_input{i}.name;
                Arm_Kinematics_out(i).Tran_matrix = eye(4);
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
