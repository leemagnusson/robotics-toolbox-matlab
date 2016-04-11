% start by analyzing ideal braking distance
m = get_motors('cell');

V = 48;
for i = 1:length(m);
    vel = V*m{i}.kv;
    b = m{i}.km^2/m{i}.R;
    tau = m{i}.J/b;
    Ts = 3*tau;
    th = m{i}.J*vel/b
    Ti = b*vel
end