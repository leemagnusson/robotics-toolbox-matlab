%% Find the closest point between to point clouds
% see title
% by Haoran Yu 3/17/2016
%%

function [p1,p2,d] = find_closest_pnt(ptcloud1,ptcloud2)
d=1000;
for i = 1:length(ptcloud1)
    for j = 1:length(ptcloud2)
    if norm(ptcloud1(:,i) - ptcloud2(:,j)) < d
        d = norm(ptcloud1(:,i) - ptcloud2(:,j));
        p1 = ptcloud1(:,i);
        p2 = ptcloud2(:,j);
    end
    end
end
end