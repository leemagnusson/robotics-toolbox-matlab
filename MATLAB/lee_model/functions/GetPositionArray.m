function q = GetPositionArray(r,u)

joint_pos = {};
for i = 1:r.n
    joint_pos{i} = linspace(u.joints{i}.limit.lower,u.joints{i}.limit.upper,2);
    if ~any(joint_pos{i}==0)
        joint_pos{i} = [joint_pos{i},0];
    end
end
joint_pos_reduced = joint_pos([1:7,10:end]);
joint_pos_all = cartesianProduct(joint_pos_reduced);
q = [joint_pos_all(:,1:7),-joint_pos_all(:,7),joint_pos_all(:,7),joint_pos_all(:,8:end)];
