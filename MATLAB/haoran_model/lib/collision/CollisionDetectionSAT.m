function collision = CollisionDetectionSAT (box1,box2)
% this funtion uses the separating axis algorithm(SAT) to detect the
% collision betwen two boxes
% box1 and box2 inputs are 8 by 3 corner points format
% collision = CollisionDetectionSAT (box1,box2)
% collision: true if colliding between two boxes
% -------------------------------------------------------
% assume collision happens
collision = true;
% get the center of the boxes
center_box1 = [mean(box1(:,1));mean(box1(:,2));mean(box1(:,3))];
center_box2 = [mean(box2(:,1));mean(box2(:,2));mean(box2(:,3))];
% vector pointing from center 1 to 2
vect_c = center_box2 - center_box1;
% define x y z axis and half width heigh depth of the box A and B
Ax = (box1(2,:)' - box1(1,:)')/norm(box1(2,:)' - box1(1,:)');
Ay = (box1(4,:)' - box1(1,:)')/norm(box1(4,:)' - box1(1,:)');
Az = (box1(5,:)' - box1(1,:)')/norm(box1(5,:)' - box1(1,:)');
WA = norm(box1(2,:)' - box1(1,:)')/2;
HA = norm(box1(4,:)' - box1(1,:)')/2;
DA = norm(box1(5,:)' - box1(1,:)')/2;
Bx = (box2(2,:)' - box2(1,:)')/norm(box2(2,:)' - box2(1,:)');
By = (box2(4,:)' - box2(1,:)')/norm(box2(4,:)' - box2(1,:)');
Bz = (box2(5,:)' - box2(1,:)')/norm(box2(5,:)' - box2(1,:)');
WB = norm(box2(2,:)' - box2(1,:)')/2;
HB = norm(box2(4,:)' - box2(1,:)')/2;
DB = norm(box2(5,:)' - box2(1,:)')/2;
% set the separating axis to be 15 different conditions and detect if there
% exists one separating axis. if there exist one separating axis, then
% collision did not happen
L = Ax;
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Ay;
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Az;
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Bx;
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = By;
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Bz;
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ax,Bx);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ax,By);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ax,Bz);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ay,Bx);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ay,By);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ay,Bz);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Az,Bx);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Az,By);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Az,Bz);
if collision
    collision = collision & (abs(dot(vect_c,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
end