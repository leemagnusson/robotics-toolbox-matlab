function [r,other_param] = create_robot(robot_dir,baseT)

dh = import_dh([robot_dir '/dh.csv']);


gravity = baseT*[0,0,-9.81,1]';

r = SerialLink(dh,'base',baseT);
r.gravity = gravity(1:3);

other_param = import_other_param([robot_dir '/other_param.csv']);


% import mass properties from solidworks files
% use solidworks evalute mass properties, select the coordinate system,
% and copy and paste to a text file
for i = 1:length(r.links)
    r.links(i) = link_add_mass_properties(r.links(i), ...
        [robot_dir '/mass/link' num2str(i) '.txt'],other_param.transmission{i});
    % comment out below to use motor inertia
    %l(i).Jm = 0;
end
