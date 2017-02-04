% set_joint_trajectory.m
% Doug Johnston
% Jan, 2017

function out_handle = set_joint_trajectory(sys, joint_block_name, have_derivatives, filter_value)
% SET_JOINT_TRAJECTORY add a block connected to the specified joint that
% allows the joint to be driven by a q vector
%
%   sys - the simulink model name
%   joint_block_name - name of joint
%   have_derivatives - if q vector has q_dot and q_dot_dot, specify true
%   filter_value - if not using derivatives, the time constant of the
%   filter, see https://www.mathworks.com/help/physmod/simscape/ref/simulinkpsconverter.html#brsaodz-1
    demux_block_lib = 'built-in/demux';
    converter_block_lib = 'nesl_utility/Simulink-PS Converter';
    num_connections = 1;
    if (have_derivatives)
        num_connections = 3;
    end
    out_handle = -1;
    joint_block = [sys '/' joint_block_name];
    jbp = get_param(joint_block, 'Position');
    demux_block_name = [joint_block_name '_demux'];
    demux_block = [sys '/' demux_block_name];
    converter_block_name = [joint_block_name '_conv'];
    converter_block = [sys '/' converter_block_name];
    % only continue if the joint is prismatic or revolute
    if (getSimulinkBlockHandle(converter_block) > 0 || ...
        (strcmp(get_param(joint_block,'ClassName'), 'RevoluteJoint') ~= 1 && ...
         strcmp(get_param(joint_block,'ClassName'), 'PrismaticJoint') ~= 1))
        return
    end
    add_block(converter_block_lib,  converter_block, 'Position', [jbp(1)-45, jbp(2)+50, jbp(1)-25, jbp(4)+50]);
    converter_block_handles = get_param(converter_block, 'PortHandles');
    set_param(converter_block,'UdotUserProvided', string(num_connections-1));
    set_param(joint_block, 'MotionActuationMode', 'InputMotion');
    set_param(joint_block, 'TorqueActuationMode', 'ComputedTorque');
    joint_block_handles = get_param(joint_block, 'PortHandles');
    add_line(sys, joint_block_handles.LConn(2), converter_block_handles.RConn(1), 'autorouting', 'on');
    if (have_derivatives)
        add_block(demux_block_lib,  demux_block, 'Position', [jbp(1)-70, jbp(2)+50, jbp(1)-65, jbp(4)+50]);
        set_param(demux_block, 'Outputs', string(num_connections));
        set_param(demux_block, 'DisplayOption', 'bar');
        set_param(converter_block,'FilteringAndDerivatives', 'provide');
        demux_block_handles = get_param(demux_block, 'PortHandles');
        for i = 1:num_connections
            add_line(sys, demux_block_handles.Outport(i), converter_block_handles.Inport(i), 'autorouting', 'on');
        end
        out_handle = demux_block_handles.Inport(1);
    else
        set_param(converter_block,'FilteringAndDerivatives', 'filter');
        set_param(converter_block,'SimscapeFilterOrder', '2');
        set_param(converter_block,'InputFilterTimeConstant', string(filter_value));
        out_handle = converter_block_handles.Inport(1);
    end
end
