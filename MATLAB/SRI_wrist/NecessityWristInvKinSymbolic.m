function [q,qd] = NecessityWristInvKinSymbolic(theta1,theta2,theta3)
load Loop1;
load Loop2;
load Loop3;
if theta1 > 0
    q(1) = Loop1.pos(1)*theta1^2+Loop1.pos(2)*theta1+Loop1.pos(3);
    q(2) = Loop1.neg(1)*theta1^2+Loop1.neg(2)*theta1+Loop1.neg(3);
    qd(1) = 2*Loop1.pos(1)*theta1 + Loop1.pos(2);
    qd(2) = 2*Loop1.neg(1)*theta1 + Loop1.neg(2);
elseif theta1 < 0
    q(1) = Loop1.neg(1)*theta1^2-Loop1.neg(2)*theta1+Loop1.neg(3);
    q(2) = Loop1.pos(1)*theta1^2-Loop1.pos(2)*theta1+Loop1.pos(3);
    qd(1) = -2*Loop1.neg(1)*theta1 - Loop1.neg(2);
    qd(2) = -2*Loop1.pos(1)*theta1 - Loop1.pos(2);
else
    q(1) = 0;
    q(2) = 0;
    qd(1) = 2.65;
    qd(2) = -2.65;
end
if theta2 > 0
    q(3) = Loop2.pos(1)*theta2^2+Loop2.pos(2)*theta2+Loop2.pos(3);
    q(4) = Loop2.neg(1)*theta2^2+Loop2.neg(2)*theta2+Loop2.neg(3);
    qd(3) = 2*Loop2.pos(1)*theta2 + Loop2.pos(2);
    qd(4) = 2*Loop2.neg(1)*theta2 + Loop2.neg(2);
elseif theta2 < 0
    q(3) = Loop2.neg(1)*theta2^2-Loop2.neg(2)*theta2+Loop2.neg(3);
    q(4) = Loop2.pos(1)*theta2^2-Loop2.pos(2)*theta2+Loop2.pos(3);
    qd(3) = -2*Loop2.neg(1)*theta2 - Loop2.neg(2);
    qd(4) = -2*Loop2.pos(1)*theta2 - Loop2.pos(2);
else
    q(3) = 0;
    q(4) = 0;
    qd(3) = -2.65;
    qd(4) = 2.65;
end
if theta3 > 0
    q(5) = Loop3.pos(1)*theta3^2+Loop3.pos(2)*theta3+Loop3.pos(3);
    q(6) = Loop3.neg(1)*theta3^2+Loop3.neg(2)*theta3+Loop3.neg(3);
    qd(5) = 2*Loop3.pos(1)*theta3 + Loop3.pos(2);
    qd(6) = 2*Loop3.neg(1)*theta3 + Loop3.neg(2);
elseif theta3 < 0
    q(5) = Loop3.neg(1)*theta3^2-Loop3.neg(2)*theta3+Loop3.neg(3);
    q(6) = Loop3.pos(1)*theta3^2-Loop3.pos(2)*theta3+Loop3.pos(3);
    qd(5) = -2*Loop3.neg(1)*theta3 - Loop3.neg(2);
    qd(6) = -2*Loop3.pos(1)*theta3 - Loop3.pos(2);
else
    q(5) = 0;
    q(6) = 0;
    qd(5) = 2.65;
    qd(6) = 2.65;
end