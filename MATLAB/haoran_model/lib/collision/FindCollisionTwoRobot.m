%% Two arm collision detection
% detect collision between two robots and save colliding arm index
% [collision,colliding_arm_index] = FindCollisionTwoRobot(point_boundary1,point_boundary2,indices_arm)
% Output:
% collision: true if colliding
% colliding_arm_index: n by 2 array. each row is the colliding arm index
% for each arm.
% Input:
% point_boundary1 and point_boundary2: cell array of bounding boxes for
% each robot
% indices_arm: arm indices that we want to detect collision. [7 8 9 10] for
% spherical arm only.
%%

function [collision,colliding_arm_index] = FindCollisionTwoRobot(point_boundary1,point_boundary2,indices_arm)
colliding_arm_index = [];
collision = false;
% detect collision
for index1 = indices_arm
    for index2 = indices_arm
        if CollisionDetectionSAT(point_boundary1{index1},point_boundary2{index2})
            colliding_arm_index = [colliding_arm_index [index1;index2]];
        end
        collision = collision | CollisionDetectionSAT(point_boundary1{index1},point_boundary2{index2});
    end
end
end