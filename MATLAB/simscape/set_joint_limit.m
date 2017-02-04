% set_joint_limit.m
% Doug Johnston
% Jan, 2017

function [stop_block] = set_joint_limit(sys, joint_block_name, lower_limit, upper_limit)
    stop_block_lib = 'multibody_3D_1D_intf_lib/Rotational Hard Stop Friction PS';

    joint_block = [sys '/' joint_block_name];
    joint_block_position = get_param(joint_block, 'Position');
    set_param(joint_block, 'SensePosition', 'on');
    set_param(joint_block, 'SenseVelocity', 'on');
    set_param(joint_block, 'TorqueActuationMode', 'InputTorque');
    stop_block_name = [joint_block_name '_stop'];
    stop_block = [sys '/' stop_block_name];
    if (getSimulinkBlockHandle(stop_block) < 0)
        add_block(stop_block_lib,  stop_block, 'Position', joint_block_position - [0 50 0 60]);
        stop_block_handles = get_param(stop_block, 'PortHandles');
        joint_block_handles = get_param(joint_block, 'PortHandles');
        add_line(sys, joint_block_handles.LConn(2), stop_block_handles.LConn(1), 'autorouting', 'on');
        add_line(sys, joint_block_handles.RConn(2), stop_block_handles.RConn(2), 'autorouting', 'on');
        add_line(sys, joint_block_handles.RConn(3), stop_block_handles.RConn(1), 'autorouting', 'on');
    end
    set_param(stop_block, 'hs_angle_upper', string(upper_limit));
    set_param(stop_block, 'hs_angle_lower', string(lower_limit));


end


function [blocks] = set_all_joint_limit(sys)
    blocks = get_param(sys, 'blocks');
    for b = blocks
        block = char(b);
        if (endsWith(block,'_joint') && strcmp(get_param([sys '/' block],'ClassName'), 'RevoluteJoint') == 1)
            set_joint_limit(sys, block, -2, 2);
        end
    end
end
