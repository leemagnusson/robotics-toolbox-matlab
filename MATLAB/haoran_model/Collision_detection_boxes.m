function no_collision = Collision_detection_boxes (box1,box2)
no_collision = true;
center_box1 = [mean(box1(:,1));mean(box1(:,2));mean(box1(:,3))];
center_box2 = [mean(box2(:,1));mean(box2(:,2));mean(box2(:,3))];
T = center_box2 - center_box1;
% Vect_1_2 = center_box2 - center_box1;
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
L = Ax;
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Ay;
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Az;
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Bx;
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = By;
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = Bz;
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ax,Bx);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ax,By);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ax,Bz);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ay,Bx);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ay,By);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Ay,Bz);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Az,Bx);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Az,By);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
L = cross(Az,Bz);
if no_collision
    no_collision = no_collision & (abs(dot(T,L)) <= ( abs(WA*dot(Ax,L)) + abs(HA*dot(Ay,L)) + abs(DA*dot(Az,L)) + abs(WB*dot(Bx,L)) + abs(HB*dot(By,L)) + abs(DB*dot(Bz,L))));
end
collision = ~no_collision;
end