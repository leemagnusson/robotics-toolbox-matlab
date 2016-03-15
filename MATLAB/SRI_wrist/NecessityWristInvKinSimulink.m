function q = NecessityWristInvKinSimulink(theta1,theta2,theta3)

Loop1_pos = [0.6540    2.9644   -0.0506];
Loop1_neg = [0.0001   -2.6391   -0.0003];
Loop2_pos = [-0.0001   -2.6388    0.0004];
Loop2_neg = [0.6208    2.7001   -0.0325];
Loop3_pos = [-0.0001   -2.6388    0.0004];
Loop3_neg = [0.6208    2.7001   -0.0325];

if theta1 > 0
    q(1) = Loop1_pos(1)*theta1^2+Loop1_pos(2)*theta1+Loop1_pos(3);
    q(2) = Loop1_neg(1)*theta1^2+Loop1_neg(2)*theta1+Loop1_neg(3);
elseif theta1 < 0
    q(1) = Loop1_neg(1)*theta1^2-Loop1_neg(2)*theta1+Loop1_neg(3);
    q(2) = Loop1_pos(1)*theta1^2-Loop1_pos(2)*theta1+Loop1_pos(3);
else
    q(1) = 0;
    q(2) = 0;
end
if theta2 > 0
    q(3) = Loop2_pos(1)*theta2^2+Loop2_pos(2)*theta2+Loop2_pos(3);
    q(4) = Loop2_neg(1)*theta2^2+Loop2_neg(2)*theta2+Loop2_neg(3);
elseif theta2 < 0
    q(3) = Loop2_neg(1)*theta2^2-Loop2_neg(2)*theta2+Loop2_neg(3);
    q(4) = Loop2_pos(1)*theta2^2-Loop2_pos(2)*theta2+Loop2_pos(3);
else
    q(3) = 0;
    q(4) = 0;
end
if theta3 > 0
    q(5) = Loop3_pos(1)*theta3^2+Loop3_pos(2)*theta3+Loop3_pos(3);
    q(6) = Loop3_neg(1)*theta3^2+Loop3_neg(2)*theta3+Loop3_neg(3);
elseif theta3 < 0
    q(5) = Loop3_neg(1)*theta3^2-Loop3_neg(2)*theta3+Loop3_neg(3);
    q(6) = Loop3_pos(1)*theta3^2-Loop3_pos(2)*theta3+Loop3_pos(3);
else
    q(5) = 0;
    q(6) = 0;
end