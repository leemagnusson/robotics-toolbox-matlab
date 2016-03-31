%% Two arm collision detection
% detect collision between two arms and save colliding arm index
% by Haoran Yu 3/28/2016
%%

function [Collision,collide_index] = find_collision_two_arm(point_boundary1,point_boundary2,indices)
collide_index = [];
Collision = false;
% detect collision
for index1 = indices
    for index2 = indices
        if Collision_detection_SAT(point_boundary1{index1},point_boundary2{index2})
            collide_index = [collide_index [index1;index2]];
        end
        Collision = Collision | Collision_detection_SAT(point_boundary1{index1},point_boundary2{index2});
    end
end
end