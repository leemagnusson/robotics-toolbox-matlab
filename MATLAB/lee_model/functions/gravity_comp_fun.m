function [ Tj ] = gravity_comp_fun( r,t,q,qd,forces,xd )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

disp(t)

Tg = r.gravload(q);
Ta = zeros(size(q));

Ke = 1000*diag(ones(size(xd)));
B = 5*eye(r.n);
Be = 200*diag(ones(size(xd)));

Jall = jacob_all(r,q);

Tx = r.fkine(q);
x = [transl(Tx);tr2rpy(Tx)'];
x_dot = r.jacob0(q)*qd';
Tv = (r.jacob0(q)'*(Ke*(xd-x) + Be*(-x_dot)))';

% external forces read from file
for j = 1:length(forces)
    if(t > forces{j}.tmin && ...
        t < forces{j}.tmax)
        n = forces{j}.joint;
        Ta(1:n) = Ta(1:n) + (Jall{n}'*forces{j}.F')';
    end
end

Tj = Tg + Ta - (B*qd')' + Tv;

end

