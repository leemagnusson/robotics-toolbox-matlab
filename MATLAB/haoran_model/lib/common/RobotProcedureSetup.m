%% Robot Procedure setup
% Automatically setup the robot with estimated joint value, table adapter
% joint value, table adapter number, procedure type and table adapter
% versison.
% [q_init_setup] = RobotProcedureSetup(q_estimate,q_bed_adapter,selected_bed_adapter,procedure_num,bed_adapter_version)
% q_estimate: 11 by n estimated joint values. n = 3 or 4 for 3 or 4 arm
% setup. In Graphic User Interface, q_estimated is assigned to be zeros,
% retracted or draping values.
% q_bed_adapter: 3 by n table adapter values
%%
function [q_init_setup] = RobotProcedureSetup(q_estimate,q_bed_adapter,selected_bed_adapter,procedure_num,bed_adapter_version)
load('urdf_info_1.0.mat')
% load('arm_version_1.0.mat')
load('index_joints.mat');
load('vertex_bed.mat');
robot_object1 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object2 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object3 = RobotClass(urdf_link_input,urdf_joint_input);
robot_object4 = RobotClass(urdf_link_input,urdf_joint_input);
robot_arms = {robot_object1;robot_object2;robot_object3;robot_object4};
if bed_adapter_version == 1
    load('vertex_bed_adapter_new.mat');
else
    load('vertex_bed_adapter.mat');
end
% init procedure setup
switch procedure_num
    case 1
        InitHerniaSetup;
    case 2
        InitThoracicSetup;
    case 3
        InitHysterectomySetup;
    case 4
        InitGastricBypassSetup;
    case 5
        InitProstatectomySetup;
    case 6
        InitLarSetup;
    case 7
        InitOmentectomySetup;
    otherwise
        error('No such procedure');
end
% init resolved rates
InitIKParameters;
num_arm = 4;
% initialize all the arms
for index_robot =1:num_arm
    if selected_bed_adapter(index_robot) ~= 0
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,selected_bed_adapter(index_robot)));
    else
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,8));
    end
    robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
    
    q(:,index_robot) = q_estimate(:,index_robot);
    robot_arms{index_robot}.CalculateFK(q(:,index_robot));
    p_rcm(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_rcm);
    rotation_rcm(:,:,index_robot) = robot_arms{index_robot}.frames_(1:3,1:3,index_car);
    p_t(:,index_robot) = translation_trocar(:,index_robot); % target position
%     rotation_t(:,:,index_robot) = rotation_trocar(:,:,index_robot); % target orientation
    rotation_t(:,:,index_robot) = eye(3); % target orientation
    
end
% Use 6Dof cartesian to move the rcm link
converged = zeros(num_arm,1);
index_robot = 0;
% bed adapter
for index_bed_adapter = selected_bed_adapter;
    index_robot = index_robot + 1;
    if index_bed_adapter ~=0
        % resolved rates
        q(:,index_robot) = robot_arms{index_robot}.InverseKinematics(q(:,index_robot),p_t(:,index_robot),rotation_t(:,:,index_robot),'Cartesian 3');
    else
        q(:,index_robot) = q(:,index_robot);
    end
end
index_robot = 0;
for index_bed_adapter = selected_bed_adapter;
    index_robot = index_robot + 1;
    if index_bed_adapter ~=0
        frames_bed_adapter = CalculateBedAdapterFK(q_bed_adapter(:,index_robot),frames_bed_adapter_base(:,:,index_bed_adapter));
        robot_arms{index_robot}.transformation_base_ = frames_bed_adapter(:,:,end);
        
        % set rcm pose and target pose
        robot_arms{index_robot}.CalculateFK(q(:,index_robot));
        p_eef(:,index_robot) = robot_arms{index_robot}.frames_(1:3,4,index_eef);
        rotation_eef(:,:,index_robot) = robot_arms{index_robot}.frames_(1:3,1:3,index_eef);
        if index_robot==2
            p_t(:,index_robot) = translation_camera;
        else
            p_t(:,index_robot) = 0.9 * translation_target + 0.1 * translation_trocar(:,index_robot); % target position
        end
        rotation_t(:,:,index_robot) = rotation_eef(:,:,index_robot); % target orientation
        % resolved rates
        q(:,index_robot) = robot_arms{index_robot}.InverseKinematics(q(:,index_robot),p_t(:,index_robot),rotation_t(:,:,index_robot),'Repositioning eef pos');
        
    else
        q(:,index_robot) = q(:,index_robot);
    end
    q_init_setup(:,index_robot) = WrapToPi(q(:,index_robot));
end