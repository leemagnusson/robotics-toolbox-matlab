%sense_joint_torque.m
% Doug Johnston
% Jan, 2017

function [ handle ] = sense_joint_torque(sys, joint_name)
%SENSE_JOINT_TORQUE add a simulink block to sense torque from the joint
    handle = -1;
    converter_block_lib = 'nesl_utility/PS-Simulink Converter';
    joint_block = [sys '/' joint_name];
    converter_block_name = [joint_name '_conv_out'];
    converter_block = [sys '/' converter_block_name];
    if (getSimulinkBlockHandle(converter_block) > 0 || ...
        (strcmp(get_param(joint_block,'ClassName'), 'RevoluteJoint') ~= 1 && ...
         strcmp(get_param(joint_block,'ClassName'), 'PrismaticJoint') ~= 1))
        return
    end

    jbp = get_param(joint_block, 'Position');
    add_block(converter_block_lib,  converter_block, 'Position', [jbp(3)+25, jbp(4)+10, jbp(3)+55, jbp(4)+50]);
    converter_block_handles = get_param(converter_block, 'PortHandles');
    set_param(joint_block, 'SenseTorqueForce', 'on');
    joint_block_handles = get_param(joint_block, 'PortHandles');
    add_line(sys, joint_block_handles.RConn(2), converter_block_handles.LConn(1), 'autorouting', 'on');
    handle = converter_block_handles.Outport(1);
end
