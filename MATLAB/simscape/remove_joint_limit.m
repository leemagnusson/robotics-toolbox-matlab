% remove_joint_limit.m
% Doug Johnston
% Jan, 2017

function [stop_block] = remove_joint_limit(sys, joint_block_name)
    joint_block = [sys '/' joint_block_name];
    stop_block_name = [joint_block_name '_stop'];
    stop_block = [sys '/' stop_block_name];
    if (getSimulinkBlockHandle(stop_block) > 0)
        stop_block_handles = get_param(stop_block, 'PortHandles');
        joint_block_handles = get_param(joint_block, 'PortHandles');
        delete_line(sys, joint_block_handles.LConn(2), stop_block_handles.LConn(1));
        delete_line(sys, joint_block_handles.RConn(2), stop_block_handles.RConn(2));
        delete_line(sys, joint_block_handles.RConn(3), stop_block_handles.RConn(1));
        delete_block(stop_block);
    end
    set_param(joint_block, 'SensePosition', 'off');
    set_param(joint_block, 'SenseVelocity', 'off');
    set_param(joint_block, 'TorqueActuationMode', 'NoTorque');
end
