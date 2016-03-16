function [p,R,J,q,Jq] = NecessityToolDirKin(theta,delta,gamma)

load WRIST;

WRIST.angles = [theta,delta+0.5*gamma,delta-0.5*gamma];

WRIST = NecessityWristDirKin(WRIST);
WRIST = NecessityWristInvKin(WRIST);
[q,dq] = NecessityWristInvKinSymbolic(WRIST.angles(1),WRIST.angles(2),WRIST.angles(3));
Jq = [dq(1) dq(3)    0
      dq(2)    0   dq(5)
      dq(2)    0   dq(6)
      dq(1) dq(4)     0];
q = [q(1)+q(3);q(2)+q(5);q(2)+q(6);q(1)+q(4)];
text(4,4,-5,'q_1','Color','r');
text(-4,4,-5,'q_2','Color','r');
text(-4,-4,-5,'q_3','Color','r');
text(4,-4,-5,'q_4','Color','r');
p = WRIST.p;
R = WRIST.R;
J1 = [1,0,0
      0,1,+0.5
      0,1,-0.5];
J = WRIST.J*J1;
%%
We = [0;1;0;0;0;0];
Wgen1 = WRIST.J_left' * We
Ftendons1 = pinv(Jq')*Wgen1;
Wgen2 = WRIST.J_right' * We
Ftendons2 = pinv(Jq')*Wgen2;
Wgen3 = WRIST.J'*We
Ftendons3 = pinv(Jq')*Wgen3;
disp([Ftendons1,Ftendons2,Ftendons3])
sum(Ftendons3)