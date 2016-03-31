% jacob_all.m
% gives jacobians at each joint frame
% adapted from Haoran's code

function J = jacob_all(r,q)

[~,Tall] = r.fkine(q);

J = {};
for i=1:r.n
    % Oi origin of joint in question (x,y,z)
    Oi = transl(Tall(:,:,i));
    
    % Alogrithm from Haoran
    Ai = r.links(i).A(q(i));
    d(:,i) = transl(Ai);
    z(:,i) = Ai(1:3,3);
    
    for j = 1:i
        J{i}(:,j) = [cross(z(:,j),Oi - d(:,j)); z(:,j)];
    end
end