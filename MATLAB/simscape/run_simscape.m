
model = 'V1_5_1_Arm_URDF_new';

% add stl path
addpath(genpath('../haoran_model/Arm_version_1.5_new'));

%q = timeseries( coupling_matrix * [q_store{arm_number};zeros(2,length(q_store{arm_number}))], ...
%    time_log(1:length(q_store{arm_number})));

q = timeseries( ones(15,2)*diag([0,1]), [0;1]);

torque = RunSimscapeInverseDynamics(model,q);

plot(torque);