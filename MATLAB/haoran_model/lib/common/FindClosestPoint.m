%% Find the closest point between to point clouds
% see title
%%

function [point1,point2,distance] = FindClosestPoint(point_clouds1,point_clouds2)
distance=1000;
for i = 1:length(point_clouds1)
    for j = 1:length(point_clouds2)
    if norm(point_clouds1(:,i) - point_clouds2(:,j)) < distance
        distance = norm(point_clouds1(:,i) - point_clouds2(:,j));
        point1 = point_clouds1(:,i);
        point2 = point_clouds2(:,j);
    end
    end
end
end