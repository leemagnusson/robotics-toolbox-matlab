
function [arm_torque] = RunSimscapeInverseDynamics(model,q)

open_system(model)
hws = get_param(model, 'modelworkspace');
hws.assignin('filter_dt',1/400);
hws.assignin('arm_q',q);
set_param(model, 'StopTime', string(q.time(end)));
sim(model);