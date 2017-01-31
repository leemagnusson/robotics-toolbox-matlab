function frames_bed_adapter = CalculateBedAdapterFK( q_bed_adapter,transformation_bed_adapter_base)
% frames_bed_adapter = CalculateBedAdapterFK( q_bed_adapter,transformation_bed_adapter_base)
% this function calculate the bed adapter FK
% Input:
% q_bed_adapter: 3 by 1 vector for bed adapter joints. first joint is
% rotation, second joint is translation, third joint is dummy(always 0)
% transformation_bed_adapter_base: 4 by 4 homogeneous transformation matrix
% representing the base frame of the bed adapter in world frame. This
% matrix is calculated from the bed urdf input and is saved in
% 'bed_adapter_info.mat'.

load([fileparts(mfilename('fullpath')) '/../../data/bed_adapter_info.mat']);

if nargin ~=0
    for i = 1:3
        translation = xyz_bed_adapter(:,i);
        euler = rpy_bed_adapter(:,i);
        joint_axis = axis_bed_adapter(:,i);
        % active joint value
        if i ~= 2
            joint_value = q_bed_adapter(i);
            rotation = RotationAxisAngle([0;0;1],euler(3))*...
                RotationAxisAngle([0;1;0],euler(2))*...
                RotationAxisAngle([1;0;0],euler(1))*...
                RotationAxisAngle(joint_axis,joint_value);
            frames_temp(:,:,i) = [rotation translation; 0 0 0 1];
        else
            joint_value = q_bed_adapter(i);
            rotation = RotationAxisAngle([0;0;1],euler(3))*...
                RotationAxisAngle([0;1;0],euler(2))*...
                RotationAxisAngle([1;0;0],euler(1));
            displacement = joint_axis * joint_value;
            frames_temp(:,:,i) = [rotation translation + rotation * displacement; 0 0 0 1];            
        end
        
    end
    % Transform the homonegeous matrices to world frame
    frames_bed_adapter(:,:,1) = transformation_bed_adapter_base;
    for i = 2:4
        frames_bed_adapter(:,:,i) = frames_bed_adapter(:,:,i-1) * frames_temp(:,:,i-1);
    end
    
end
end
