% look at static joint loading case provided by Karen

% 10 N at .28m out
% 18 N at remote center
% 4Nm at remote center

ncases = 1*6*4*4;

qout = [-2.5,0,.9,0,1,0,1.2,-.28,0];
qin = qout;
qin(8) = 0;
figure(10);
r.tool = transl(0,0,0);
r.plot3d(qout,'workspace',[-1,1,-1,1,-.5,1]);


% gravity
Tg = r.gravload(qout)';




% first 10 N
F10 = zeros(6,6);
F10(1:3,:) = [10,-10,0,0,0,0;
              0,0,10,-10,0,0;
              0,0,0,0,10,-10];

%r.tool = transl(0,0,0);
J = r.jacobn(qout);
T10 = J'*F10;

% then 18N
F18 = zeros(6,4);
F18(1:2,:) = 1*[10,-10,0,0;
              0,0,10,-10];
%r.tool = transl(0,0,.28);
J = r.jacobn(qin);
T18 = J'*F18;

% 4Nm
F4 = zeros(6,4);
F4(4:5,:) = [4,-4,0,0;
             0,0,4,-4];
T4 = J'*F4;         

Ttotal = zeros(size(Tg,1),6,4,4);
for i = 1:6
    for j = 1:4
        for k = 1:4
            Ttotal(:,i,j,k) = Tg+T10(:,i)+T18(:,j)+T4(:,k);
        end
    end
end


Ttotalmax = max(max(max(abs(Ttotal),[],2),[],3),[],4)