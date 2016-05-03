% specify_stiffness.m

r.links(11).m = 2; % design for heavy driver
% desired frequency
wdes = 20*2*pi*ones(1,r.n);

joint_pos = {};
for i = 1:r.n
    joint_pos{i} = linspace(u.joints{i}.limit.lower,u.joints{i}.limit.upper,2);
    if ~any(joint_pos{i}==0)
        joint_pos{i} = [joint_pos{i},0];
    end
end
joint_pos_reduced = joint_pos([1:7,10:end]);
joint_pos_all = cartesianProduct(joint_pos_reduced);
joint_pos_all = [joint_pos_all(:,1:7),-joint_pos_all(:,7),joint_pos_all(:,7),joint_pos_all(:,8:end)];

kdiag = zeros(r.n,length(joint_pos_all));

parfor i = 1:length(joint_pos_all)
    if (mod(i,1000)==0)
        disp(i/1000);
    end
    q = joint_pos_all(i,:);
    J = r.jacob0(q);
    M = r.inertia(q);

    A = zeros(r.n);
    K = -place(A,inv(M),wdes.^2);
    kdiag(:,i) = diag(K);
    
end

Kmax = max(kdiag,[],2)