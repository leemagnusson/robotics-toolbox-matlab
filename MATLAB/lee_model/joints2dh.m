% Lee Magnusson 3/16/16

% todo add prismatic support
function [dh,T_mod,T] = joints2dh(j)
% Convert urdf style rpy and xyz values to DH
%   Use just the location of the axes to determine new coordinate systems
%   for dh

n = length(j);calc
dh = zeros(n,6);


for i = 1:n
    rpy = j{i}.origin.rpy;
    Roriginal(:,:,i) = rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1));
    Toriginal(:,:,i) = r2t(Roriginal(:,:,i));
    Toriginal(1:3,4,i) = j{i}.origin.xyz;    
end

T_mod(:,:,1) = eye(4);
T(:,:,1) = Toriginal(:,:,1);
% todo need to keep track of origins
for i = 2:n
    % T_mod goes from urdf coordinate system to dh coordinate system
 %   T_mod(:,:,i) = T(:,:,i-1)*T_mod(:,:,i-1)*inv(Toriginal(:,:,i-1));
    % T_tmp goes from dh coordinate system to next urdf coordinate system
    T_tmp = inv(T_mod(:,:,i-1))*Toriginal(:,:,i);
    
    % zi is unit vector in z direction
    z(i,:) = t2r(T_tmp)*j{i}.axis.xyz';
    z(i,:) = z(i,:)/norm(z(i,:));
    
    % unit vector in common normal direction
    cn(i,:) = cross([0,0,1],z(i,:));
    if(norm(cn(i,:)) < 1e-5)
        % lines are parallel, cn is xy distance between origins
        parallel = true;
        cn(i,:) = [T_tmp(1:2,4);0];
    else
        parallel = false;
    end
    cn(i,:) = cn(i,:)/norm(cn(i,:));

    % Ax = b, x is [a,d,beta]
    if (parallel)
        a(i) = sqrt(Toriginal(1,4,i)^2 + ...
                    Toriginal(2,4,i)^2);
        d(i) = 0;
        
    else
        A = [cn(i,:)',[0;0;1],-z(i,:)'];
        b = T_tmp(1:3,4);
        x = A\b;
        a(i) = x(1);
        d(i) = x(2);
        beta(i) = x(3);
        actual_origin(i,:) = (b + beta(i)*z(i,:)')';
    end

    th(i) = acos(dot([1,0,0],cn(i,:)));
    signth(i) = sign(dot(cross([1,0,0],cn(i,:)),[0,0,1]));
    th(i) = signth(i)*th(i);
    
    alpha(i) = acos(dot([0,0,1],z(i,:)));
    signalp(i) = sign(dot(cross([0,0,1],z(i,:)),cn(i,:)));
    alpha(i) = signalp(i)*alpha(i);
    
    T(:,:,i) = transl(0,0,d(i))*r2t(rotz(th(i)))*transl(a(i),0,0)*r2t(rotx(alpha(i)));
    T_mod(:,:,i) = inv(Toriginal(:,:,i))*T_mod(:,:,i-1)*T(:,:,i);
end

dh(:,2) = d;
dh(:,3) = a;
dh(:,4) = alpha;
dh(:,6) = th;