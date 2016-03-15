function WRIST = NecessityWristDirKin(WRIST)
%% import STL files

alpha = WRIST.angles(1);
beta = WRIST.angles(2);
gamma = WRIST.angles(3);

figure(1);
cla;
hold on;
draw_coordinate_system([10 10 10],eye(3),[0;0;0],['r' 'g' 'b'],'0');
h1 = trisurf(WRIST.bodies{1}.t,WRIST.bodies{1}.p(1,:),WRIST.bodies{1}.p(2,:),WRIST.bodies{1}.p(3,:),'FaceColor','y','FaceAlpha',1);
WRIST.Origins{1} = zeros(3,1);
WRIST.Orientations{1} = eye(3);
R01 = RotationMatrix_rad(alpha,[0;1;0]);
WRIST.Orientations{2} = R01;
p01 = [0;0;0];
WRIST.Origins{2} = p01;
WRIST.bodies{2}.p = R01*WRIST.bodies{2}.p;
draw_coordinate_system([10 10 10],R01,[0;0;0],['r' 'g' 'b'],'1');
h2 = trisurf(WRIST.bodies{2}.t,WRIST.bodies{2}.p(1,:),WRIST.bodies{2}.p(2,:),WRIST.bodies{2}.p(3,:),'FaceColor','g','FaceAlpha',1);
R02 = R01*RotationMatrix_rad(beta,[1;0;0]);
WRIST.Orientations{3} = R02;
p02 = p01 + R01*[0;0;12.33];
WRIST.Origins{3} = p02;
draw_coordinate_system([10 10 10],R02,p02,['r' 'g' 'b'],'2');
p_pad = [p02(1)*ones(1,size(WRIST.bodies{3}.p,2))
         p02(2)*ones(1,size(WRIST.bodies{3}.p,2))
         p02(3)*ones(1,size(WRIST.bodies{3}.p,2))];
WRIST.bodies{3}.p = p_pad + R02*WRIST.bodies{3}.p;
h3 = trisurf(WRIST.bodies{3}.t,WRIST.bodies{3}.p(1,:),WRIST.bodies{3}.p(2,:),WRIST.bodies{3}.p(3,:),'FaceColor','c');
R03 = R01*RotationMatrix_rad(gamma,[1;0;0]);
WRIST.Orientations{4} = R03;
p03 = p02;
WRIST.Origins{4} = p03;
draw_coordinate_system([10 10 10],R03,p02,['r' 'g' 'b'],'3');
p_pad = [p03(1)*ones(1,size(WRIST.bodies{4}.p,2))
         p03(2)*ones(1,size(WRIST.bodies{4}.p,2))
         p03(3)*ones(1,size(WRIST.bodies{4}.p,2))];
WRIST.bodies{4}.p = p_pad + R03*WRIST.bodies{4}.p;
h4 = trisurf(WRIST.bodies{4}.t,WRIST.bodies{4}.p(1,:),WRIST.bodies{4}.p(2,:),WRIST.bodies{4}.p(3,:),'FaceColor','c');
WRIST.Origins{4} = p03;
p_pad = [p02(1)*ones(1,size(WRIST.bodies{5}.p,2))
         p02(2)*ones(1,size(WRIST.bodies{5}.p,2))
         p02(3)*ones(1,size(WRIST.bodies{5}.p,2))];
WRIST.bodies{5}.p = p_pad + R02*WRIST.bodies{5}.p;
h5 = trisurf(WRIST.bodies{5}.t,WRIST.bodies{5}.p(1,:),WRIST.bodies{5}.p(2,:),WRIST.bodies{5}.p(3,:),'FaceColor','r');
p_pad = [p03(1)*ones(1,size(WRIST.bodies{6}.p,2))
         p03(2)*ones(1,size(WRIST.bodies{6}.p,2))
         p03(3)*ones(1,size(WRIST.bodies{6}.p,2))];
WRIST.bodies{6}.p = p_pad + R03*WRIST.bodies{6}.p;
h6 = trisurf(WRIST.bodies{6}.t,WRIST.bodies{6}.p(1,:),WRIST.bodies{6}.p(2,:),WRIST.bodies{6}.p(3,:),'FaceColor','r');
R04 = R01*RotationMatrix_rad(beta/2,[1;0;0])*RotationMatrix_rad(gamma/2,[1;0;0]);
WRIST.R = R04;
p04 = p02 + R04*[0;0;25];
WRIST.p = p04;
draw_coordinate_system([10 10 10],R04,p04,['r' 'g' 'b'],'ee');
%% Compute Jacobian
p_left = p02 + R02*[0;0;25];
text(p_left(1),p_left(2),p_left(3),'L');
p_right = p03 + R03*[0;0;25];
text(p_right(1),p_right(2),p_right(3),'R');
WRIST.J = [cross(R01(:,2),p04),0.5*cross(R02(:,1),p04-p02),0.5*cross(R03(:,1),p04-p02)
                 R01(:,2),R02(:,1),R03(:,1)];
WRIST.J_left = [cross(R01(:,2),p_left),cross(R02(:,1),p_left-p02),zeros(3,1)
                R01(:,2),R02(:,1),zeros(3,1)];
WRIST.J_right = [cross(R01(:,2),p_right),zeros(3,1),cross(R03(:,1),p_right-p03)
                     R01(:,2),zeros(3,1),R03(:,1)];