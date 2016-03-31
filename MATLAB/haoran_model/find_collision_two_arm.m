function [Collision,collide_index] = find_collision_two_arm(point_boundary1,point_boundary2,indices)
collide_index = [];
Collision = false;
for index1 = indices
    for index2 = indices
        if Collision_detection_boxes(point_boundary1{index1},point_boundary2{index2})
            collide_index = [collide_index [index1;index2]];
        end
        Collision = Collision | Collision_detection_boxes(point_boundary1{index1},point_boundary2{index2});
    end
end
end