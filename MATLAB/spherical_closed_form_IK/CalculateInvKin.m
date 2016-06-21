function [sol_q, valid_sol_count] = CalculateInvKin( T06_t , q_lb , q_ub , options)
% calculateInvKin calculate the inverse kinematics solution for the 1.5 arm
% Input arguments:
% T06_t = 4x4 target pose for the end effector in the base frame
% q_lb  = 6x1 lower bound on q
% q_ub  = 6x1 upper bound on q
% options: options have various fields
% kDHConstants = DH constants, [alpha2, alpha3 , d4 , lw] 
% range_check = flag that tells the function to check the range or not,
% range_check ==1 => only return solutions that are in range otherwise
% return all solutions
% offset  = is the offset in the joints at zero pose
%
% Outputs:
% valid_sol_count : number of solutions found (If range_check==1 then counts only
% the valid solutions otherwise all solutions
% sol_q : 6xsol_count where each column is a solution
%
% solutions only work for the case of this strcuture for DH paramters
%list the modified DH parameters
% al = [0 al2 al3 0 pi/2 pi/2];
% a  = [0 0 0 0 0 lw];
% d  = [0 0 q3 d4 0 0];
% th = [q1 q2 0 q4 q5 q6];
% this code is not optimized for speed, It is designed to obtain all
% possible solutions and check the answers against the original solution.
% In real-time implementaion on robot, values such as cos(al2), sin(q(5)), etc. need
% to be precalculated. 
N = 6;

%numerical values of the constants
al2 = options.kDHConstants(1);
al3 = options.kDHConstants(2);
d4  = options.kDHConstants(3);
lw  = options.kDHConstants(4);

%offsets is joint angles
q_offset = options.q_offset;

T60_t = CalculateInvT(T06_t);%base frame defined in end effector frame
Prcm = T60_t(1:3,4);%Prcm described in frame 6
Prcm2 = [Prcm(1:2);0];%projection of Prcm on the XY plane of frame 6 described in frame 6
if(abs(norm(Prcm2)) < eps(10))
    disp('infinite solution for P5, not feasible');
    error('check that the starting pose was inside the joint limits and lw~=0');
end

%two possible solutions for origin of frame 5, both described in frame 6
P5(:,1) = lw*Prcm2/norm(Prcm2);
P5(:,2) = -P5(:,1);

%OPTIONAL(for omitting some of the solutions): 5 in frame 6 should have 
% zero or negative x (because jaw cannot open past 90 deg, when the jaw is
% at 90 degrees then both solutions give equal x=0
% we have to rely on q5 to resolve that, one will result in q5 of 180
% degrees
%%uncomment this code if you want to take out one of P5s based on the above
%%--------------------------
% if P5(1,1) >= -eps(100) && P5(1,1) <= eps(100) % does not really catch it here due to numerical error
%     disp('Two solutions for P5.')
% elseif P5(1,1) > 0
%     disp('first P5 is acceptable')
%     P5(:,2) = [];
% 
% elseif P5(1,2) > 0
%     disp('second P5 is acceptable')
%     P5(:,1) = [];
% end
%%--------------------------

[~,nP5] = size(P5);%number of solutions for P5
P5in0 = zeros(4,nP5);
sol(2*nP5).q = zeros(6,1); %preallocate size of solution 
%for each P5 there are two solutions possible for q3
for i=1:nP5
    P5in0(:,i) = T06_t*[P5(:,i);1]; %P5 in frame 0
    sol(2*i-1).q(3) = -d4 + norm(P5in0(1:3,i)); %1st solution
    sol(2*i-1).path(1) = 2*i-1;
    sol(2*i-1).P5in0 = P5in0(1:3,i);
    sol(2*i).q(3) = -d4 - norm(P5in0(1:3,i)); %2nd solution
    sol(2*i).path(1) = 2*i;
    sol(2*i).P5in0 = P5in0(1:3,i);
end
sol_count = length(sol); % number of solutions
invalid_index = zeros(sol_count,1);

%populate q2, if not feasible remove the entire solution
for i=1:sol_count
    cosq2 = (cos(al2)*cos(al3) - sol(i).P5in0(3)/(d4+sol(i).q(3)))*1/(sin(al2)*sin(al3));
    if abs(cosq2)>1
        invalid_index(i) = 1;
        sol(i).path(2) = -1;
    else
        sol = [sol sol(i)]; %Duplicate the ith solution at the end
        invalid_index = [invalid_index ; -1];
        sol(i).q(2) = acos(cosq2);
        sol(i).path(2) = 1;
        invalid_index(i) = 0;
        sol(end).q(2)= -acos(cosq2);
        sol(end).path(2) = 2;
        invalid_index(end) = 0;
    end
end

sol(invalid_index == 1)=[];%remove the whole solution for invalid index
sol_count = length(sol); % number of solutions

for i=1:sol_count
    B1 = cos(al3)*sin(al2)+cos(al2)*cos(sol(i).q(2))*sin(al3);
    B2 = sin(al3)*sin(sol(i).q(2));
    B = (d4+sol(i).q(3))*[B1 B2;B2 -B1];
    %TODO: Determine if the matrix can be singular
    C= inv(B)\sol(i).P5in0(1:2); %[sin(q1) ; cos(q1)]
    sol(i).q(1) = atan2(C(1),C(2));
end

sol_count = length(sol);

for i=1:sol_count
    %calculate T30 substituting the solutions for q1 q2 q3 already found,
    %in real-time implementation this can be calculated once and just
    %evaluated
    %T30 = vpa(subs(CalculateTF(T, 3, 0),[q1 q2 q3 al2 al3 d4 lw], [sol(i).q(1) sol(i).q(2) sol(i).q(3) al2 al3 d4 lw]),VPA_N);
    q1 = sol(i).q(1);
    q2 = sol(i).q(2);
    q3 = sol(i).q(3);
    T30_t = [[                                               cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2),                                               cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2),                              sin(al2)*sin(q2),   0]
        [ sin(al2)*sin(al3)*sin(q1) - cos(al3)*cos(q1)*sin(q2) - cos(al2)*cos(al3)*cos(q2)*sin(q1), cos(al2)*cos(al3)*cos(q1)*cos(q2) - cos(al3)*sin(q1)*sin(q2) - cos(q1)*sin(al2)*sin(al3), cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2),   0]
        [ cos(al3)*sin(al2)*sin(q1) + cos(q1)*sin(al3)*sin(q2) + cos(al2)*cos(q2)*sin(al3)*sin(q1), sin(al3)*sin(q1)*sin(q2) - cos(al3)*cos(q1)*sin(al2) - cos(al2)*cos(q1)*cos(q2)*sin(al3), cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3), -q3]
        [                                                                                        0,                                                                                        0,                                             0,   1]];
    
    %now given T30 and T06 find T36
    T36_t = T30_t*T06_t;
    %calculate q5, only one solution
    sq5 = 1/lw*(d4 - T36_t(3,4));
    cq5 = -T36_t(3,3);
    sol(i).q(5) = atan2(sq5, cq5);
    %calculate q4, two cases but only one solution
    if abs(cq5)<eps(1e5) %means c5 = 0
        sol(i).q(4) = atan2(T36_t(2,3)/sq5,T36_t(1,3)/sq5);
        sol(i).path(3) = 1;
    else
        sol(i).q(4) = atan2(T36_t(2,4)/(-lw*cq5) , T36_t(1,4)/(-lw*cq5));
        sol(i).path(3) = 2;
    end
    
    %compute q6, two cases but only one solution
    % T56 = T50*T06 (both are known now) =
    %[ cos(q6), -1.0*sin(q6),    0, -1.0*lw]
    %[       0,            0, -1.0,       0]
    %[ sin(q6),      cos(q6),    0,       0]
    %[       0,            0,    0,     1.0]
    if abs(sq5) < eps(1e5)
        %T50 = vpa(subs(CalculateTF(T, 5, 0),[q1 q2 q3 q4 q5 al2 al3 d4 lw], [sol(i).q(1) sol(i).q(2) sol(i).q(3) sol(i).q(4) sol(i).q(5) al2 al3 d4 lw]),VPA_N);
        T50_t = [[ cos(q1)*cos(q2)*cos(q4)*cos(q5) + cos(al3)*sin(al2)*sin(q1)*sin(q5) + cos(q1)*sin(al3)*sin(q2)*sin(q5) - cos(al2)*cos(q4)*cos(q5)*sin(q1)*sin(q2) - cos(al3)*cos(q1)*cos(q5)*sin(q2)*sin(q4) + cos(al2)*cos(q2)*sin(al3)*sin(q1)*sin(q5) + cos(q5)*sin(al2)*sin(al3)*sin(q1)*sin(q4) - cos(al2)*cos(al3)*cos(q2)*cos(q5)*sin(q1)*sin(q4), cos(q2)*cos(q4)*cos(q5)*sin(q1) - cos(al3)*cos(q1)*sin(al2)*sin(q5) + sin(al3)*sin(q1)*sin(q2)*sin(q5) + cos(al2)*cos(q1)*cos(q4)*cos(q5)*sin(q2) - cos(al2)*cos(q1)*cos(q2)*sin(al3)*sin(q5) - cos(q1)*cos(q5)*sin(al2)*sin(al3)*sin(q4) - cos(al3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) + cos(al2)*cos(al3)*cos(q1)*cos(q2)*cos(q5)*sin(q4), cos(al2)*cos(al3)*sin(q5) + cos(al2)*cos(q5)*sin(al3)*sin(q4) + cos(q4)*cos(q5)*sin(al2)*sin(q2) - cos(q2)*sin(al2)*sin(al3)*sin(q5) + cos(al3)*cos(q2)*cos(q5)*sin(al2)*sin(q4), -sin(q5)*(d4 + q3)]
            [ cos(al3)*cos(q5)*sin(al2)*sin(q1) - cos(q1)*cos(q2)*cos(q4)*sin(q5) + cos(q1)*cos(q5)*sin(al3)*sin(q2) + cos(al2)*cos(q2)*cos(q5)*sin(al3)*sin(q1) + cos(al2)*cos(q4)*sin(q1)*sin(q2)*sin(q5) + cos(al3)*cos(q1)*sin(q2)*sin(q4)*sin(q5) - sin(al2)*sin(al3)*sin(q1)*sin(q4)*sin(q5) + cos(al2)*cos(al3)*cos(q2)*sin(q1)*sin(q4)*sin(q5), cos(q5)*sin(al3)*sin(q1)*sin(q2) - cos(q2)*cos(q4)*sin(q1)*sin(q5) - cos(al3)*cos(q1)*cos(q5)*sin(al2) - cos(al2)*cos(q1)*cos(q2)*cos(q5)*sin(al3) - cos(al2)*cos(q1)*cos(q4)*sin(q2)*sin(q5) + cos(q1)*sin(al2)*sin(al3)*sin(q4)*sin(q5) + cos(al3)*sin(q1)*sin(q2)*sin(q4)*sin(q5) - cos(al2)*cos(al3)*cos(q1)*cos(q2)*sin(q4)*sin(q5), cos(al2)*cos(al3)*cos(q5) - cos(q2)*cos(q5)*sin(al2)*sin(al3) - cos(al2)*sin(al3)*sin(q4)*sin(q5) - cos(q4)*sin(al2)*sin(q2)*sin(q5) - cos(al3)*cos(q2)*sin(al2)*sin(q4)*sin(q5), -cos(q5)*(d4 + q3)]
            [                                                                                                                                                            cos(q1)*cos(q2)*sin(q4) + cos(al3)*cos(q1)*cos(q4)*sin(q2) - cos(q4)*sin(al2)*sin(al3)*sin(q1) - cos(al2)*sin(q1)*sin(q2)*sin(q4) + cos(al2)*cos(al3)*cos(q2)*cos(q4)*sin(q1),                                                                                                                                                            cos(q2)*sin(q1)*sin(q4) + cos(q1)*cos(q4)*sin(al2)*sin(al3) + cos(al2)*cos(q1)*sin(q2)*sin(q4) + cos(al3)*cos(q4)*sin(q1)*sin(q2) - cos(al2)*cos(al3)*cos(q1)*cos(q2)*cos(q4),                                                                                         sin(al2)*sin(q2)*sin(q4) - cos(al2)*cos(q4)*sin(al3) - cos(al3)*cos(q2)*cos(q4)*sin(al2),                  0]
            [                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                0,                  1]];
        T56_t = T50_t*T06_t;
        sol(i).q(6) = atan2(T56_t(3,1),T56_t(1,1));
        sol(i).path(4) = 1;
    else
        sol(i).q(6) = atan2(-T36_t(3,2)/sin(sol(i).q(5)) , T36_t(3,1)/sin(sol(i).q(5)));
        sol(i).path(4) = 2;
    end
end

for i=1:sol_count
    q1 = sol(i).q(1);
    q2 = sol(i).q(2);
    q3 = sol(i).q(3);
    q4 = sol(i).q(4);
    q5 = sol(i).q(5);
    q6 = sol(i).q(6);
    %sol(i).T06 = vpa(subs(CalculateTF(T, 0, 6),[q1 q2 q3 q4 q5 q6 al2 al3 d4 lw], [sol(i).q(1) sol(i).q(2) sol(i).q(3) sol(i).q(4) sol(i).q(5) sol(i).q(6) al2 al3 d4 lw]),VPA_N);
    sol(i).T06_c = [[ sin(q6)*(sin(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)) + cos(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1))) + cos(q6)*(sin(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - cos(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)))), cos(q6)*(sin(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)) + cos(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1))) - sin(q6)*(sin(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - cos(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)))), - sin(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2))) - cos(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)), d4*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - lw*(sin(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - cos(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)))) + q3*sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + q3*cos(al3)*sin(al2)*sin(q1)]
        [ sin(q6)*(sin(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) + cos(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + sin(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2))), cos(q6)*(sin(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) + cos(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + sin(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2))),   sin(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) - cos(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2)), d4*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2)) - lw*(cos(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + sin(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2))) + q3*sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - q3*cos(al3)*cos(q1)*sin(al2)]
        [                                                                                                                                                                     cos(q6)*(sin(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) + cos(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2))) - sin(q6)*(cos(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) - sin(al2)*sin(q2)*sin(q4)),                                                                                                                                                                   - cos(q6)*(cos(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) - sin(al2)*sin(q2)*sin(q4)) - sin(q6)*(sin(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) + cos(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2))),                                                                                                       sin(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2)) - cos(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)),                                                                                                                                                                             d4*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) - lw*(sin(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) + cos(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2))) + q3*cos(al2)*cos(al3) - q3*cos(q2)*sin(al2)*sin(al3)]
        [                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                  1]];
    
    sol(i).err = abs(sol(i).T06_c - T06_t);
    if max(max(sol(i).err))>.00001
        str = sprintf(' %f ', sol(i).q);
        error('error is too large: %s \nerror is %f ',str , max(max(sol(i).err)));
    end
end

%convert back to the raw joint angles
for i=1:sol_count
    for j=1:N
        sol(i).q_raw(j) = sol(i).q(j)-q_offset(j); %q_raw matches the unoffset angles
    end
end

%check the joint angles and if in range add them to the solution
valid_sol_count = 0; % number of valid solutions
sol_q = nan;
if options.range_check == 1
    for i=1:sol_count
        if (all(sol(i).q_raw - q_ub <= 0) && all(sol(i).q_raw - q_lb >=0))
            valid_sol_count = valid_sol_count +1;
            sol_q(valid_sol_count,1:N)= sol(i).q_raw;
        end
    end
else
    for i=1:sol_count
        sol_q(i,1:N) = sol(i).q_raw;
    end
    valid_sol_count = sol_count;
end

end