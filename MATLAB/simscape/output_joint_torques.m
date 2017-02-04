% output_joint_torques.m
% Doug Johnston
% Jan, 2017

function [ output_args ] = output_joint_torques(sys, joints, torque_name)
%OUTPUT_JOINT_TORQUES adds a block to output torque for specified joints
%   torque_name - a string representing the matlab variable to output to
    handles = [];
    p = [50 400];
    for joint = joints
        handles = [handles sense_joint_torque(sys, joint{1})];
    end
    output_block_handle = add_block('built-in/ToWorkspace',[sys '/torque_out'], 'Position', [p(1)+20, p(2), p(1)+70, p(2)+30]);
    set_param(output_block_handle, 'MaxDataPoints', 'inf');
    set_param(output_block_handle, 'SaveFormat', 'Timeseries');
    mux_block_handle = add_block('built-in/mux', [sys, '/torque_mux'], 'Position', [p(1) p(2) p(1)+5 p(2)+30]);
    set_param(output_block_handle, 'VariableName', torque_name);
    set_param(mux_block_handle, 'Inputs', string(length(handles)));
    set_param(mux_block_handle, 'DisplayOption', 'bar');
    mux_port_handles = get_param(mux_block_handle, 'PortHandles');
    output_port_handles = get_param(output_block_handle, 'PortHandles');
    add_line(sys, mux_port_handles.Outport(1), output_port_handles.Inport(1), 'autorouting', 'on');
    for h = 1:length(handles)
        if h ~= -1
            add_line(sys, handles(h), mux_port_handles.Inport(h), 'autorouting', 'on');
        end
    end
end
