%% 
% Parameter a
function WRIST = NecessityWristInvKin(WRIST)
a = WRIST.parameters.a;
b = WRIST.parameters.b;
c = WRIST.parameters.c;
d = WRIST.parameters.d;
e = WRIST.parameters.e;
f = WRIST.parameters.f;
g = WRIST.parameters.g;
figure(1);
R01 = WRIST.Orientations{2};
% transform points
P = computeLoop1(R01,a,b,c,d,e,f,g);
% calcultae convex hull
K = convhull([P(1,:);P(3,:)]');
line(P(1,K'),P(2,K'),P(3,K'),'LineWidth',3,'Color','m');
% calculate cable lengths
[L1,L2] = computeCableLengths(K,P);
%--------------------------------------------------------------------------
R02 = WRIST.Orientations{3};
p02 = WRIST.Origins{3};
P2 = computeLoop2(R01,R02,a,b,c,d,e,f,g);
K2 = convhull([P2(2,:);P2(3,:)]');
p02_pad = [p02(1)*ones(1,size(P2,2))
           p02(2)*ones(1,size(P2,2))
           p02(3)*ones(1,size(P2,2))];
P2 = p02_pad + R01*P2;
plot3(P2(1,:),P2(2,:),P2(3,:),'r*');
line(P2(1,K2'),P2(2,K2'),P2(3,K2'),'LineWidth',3,'Color','r');
[L3,L4] = computeCableLengths(K2,P2);
%--------------------------------------------------------------------------
R03 = WRIST.Orientations{4};
p03 = WRIST.Origins{4};
P3 = computeLoop3(R01,R03,a,b,c,d,e,f,g);
K3 = convhull([P3(2,:);P3(3,:)]');
p03_pad = [p03(1)*ones(1,size(P3,2))
           p03(2)*ones(1,size(P3,2))
           p03(3)*ones(1,size(P3,2))];
P3 = p03_pad + R01*P3;
plot3(P3(1,:),P3(2,:),P3(3,:),'b*');
line(P3(1,K3'),P3(2,K3'),P3(3,K3'),'LineWidth',3,'Color','b');
[L5,L6] = computeCableLengths(K3,P3);
%--------------------------------------------------------------------------
WRIST.L = [L1,L2,L3,L4,L5,L6];