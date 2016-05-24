%% Find the closest point between to point clouds
% [point1,point2,distance] = FindClosestPoint(point_clouds1,point_clouds2)
% Input:
% point_clouds1 and point_clouds2: two separate point clouds with format 3
% by n matrix
% output: 
% point1 and point2: two closest points from each input point cloud
% respectively with 3 by 1 format
% distance: distance between point1 and point2
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