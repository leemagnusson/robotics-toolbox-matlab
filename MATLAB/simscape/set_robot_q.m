% set_robot_q.m
% Doug Johnston
% Jan, 2017

function [ output_args ] = set_robot_q(sys, joints, q_name, filter_dt)
%SET_ROBOT_Q Add blocks to the simscape model to connect joint trajectories
%to each joint
%   sys - the simulink model name
%   q_name - an (N+1)xM matrix of joint angles of the form [t J1 J2 ... Jn],
%     where the first column is time
%   joints - an N length vector of joint names (as cells)
%   filter_dt - temporal size of derivative filter
%      (see https://www.mathworks.com/help/physmod/simscape/ref/simulinkpsconverter.html#brsaodz-1)
%
%   This function will create a block to feed, in order, each joint in
%   `joints` the input trajectory given in `q_name`

    handles = [];
    p = [30 20];
    for joint = joints
        handles = [handles set_joint_trajectory(sys, joint{1}, false, filter_dt)];
    end
    input_block_handle = add_block('built-in/FromWorkspace',[sys '/q_in'], 'Position', [p(1), p(2), p(1)+30, p(2)+30]);
    demux_block_handle = add_block('built-in/demux', [sys, '/q_demux'], 'Position', [p(1)+50 p(2) p(1)+55 p(2)+30]);
    set_param(input_block_handle, 'VariableName', q_name);
    set_param(demux_block_handle, 'Outputs', string(length(handles)));
    demux_port_handles = get_param(demux_block_handle, 'PortHandles');
    input_port_handles = get_param(input_block_handle, 'PortHandles');
    add_line(sys, input_port_handles.Outport(1), demux_port_handles.Inport(1), 'autorouting', 'on');
    for h = 1:length(handles)
        add_line(sys, demux_port_handles.Outport(h), handles(h), 'autorouting', 'on');
    end
end
