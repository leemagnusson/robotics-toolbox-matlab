%test the inverse kinematics
clc;
close all;
clear;
%The Variable Precision Arithmetic digits
VPA_N = 25;
%error tolerance on solutions
kTol = eps(1e8);
%number of tests to run
num_tests = 1e5;
%flag to IK to eliminate the solutions that are outside joint limit
options.range_check = 0;

%range of joint angles for random pose generation,
%q_lb is lower bound
q_lb = [-pi+pi/10 , -1.0236 , -0.092 , -pi+pi/10  , -pi/2 , -pi/2];
%q_ub is upper bound
q_ub = [pi-pi/10  , 1.4198  ,  0.167 ,  pi-pi/10  ,  pi/2 ,  pi/2];
% offset in joint angles
q_offset = [deg2rad(-10.68) , deg2rad(86.24) , 0 , 0 , pi/2 , 0];
options.q_offset = q_offset;
%other DH table constants
al2 = deg2rad(110.41);
al3 = deg2rad(100);
d4 = 0.079604;
lw = 0.01233;
options.kDHConstants = [al2 al3 d4 lw];

parfor m=1:num_tests
    %numerical values for joint angles
    q_raw = GenerateRandomJoints(q_lb , q_ub);

    % create q1 to q6 for symbolic evaluation of forward kinematics
    q1 = q_raw(1)+q_offset(1);
    q2 = q_raw(2)+q_offset(2);
    q3 = q_raw(3)+q_offset(3);
    q4 = q_raw(4)+q_offset(4);
    q5 = q_raw(5)+q_offset(5);
    q6 = q_raw(6)+q_offset(6);

    % This is how the transformations are calculated, The code below has the
    % symbolic results for the forward kinematics.
    % calculate the series of transformations Ti-1 to i (i.e. T01, T12, etc)
    % for i=1:N
    %     %Ti-1 to i in symbolic form
    %     T(:,:,i)  =     [       cos(th(i))            -sin(th(i))          0            -a(i)    ;
    %         sin(th(i))*cos(al(i)) cos(th(i))*cos(al(i)) -sin(al(i)) -sin(al(i))*d(i);
    %         sin(th(i))*sin(al(i)) cos(th(i))*sin(al(i))  cos(al(i))  cos(al(i))*d(i);
    %         0                      0               0               1     ];
    %     %Ti-1 to i evaluated at nominal values
    %     Ts(:,:,i) = vpa(subs(T(:,:,i), [q1 q2 q3 q4 q5 q6 al2 al3 d4 lw] ,[q1s q2s q3s q4s q5s q6s al2s al3s d4s lws] ),VPA_N);
    % end
    % T = vpa(T,6);
    % Tij the matrix that describes frame j in frame i
    % T06 = CalculateTF(T,0,6); %end effector in frame 0

    % symbolic form of the transformation from end effector to base
    % This is just for demonstration purpose. In reality this will be given
    % from external inputs and there is no need to explicitly calculate it.
    transformation_eef =[[ sin(q6)*(sin(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)) + cos(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1))) + cos(q6)*(sin(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - cos(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)))), cos(q6)*(sin(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)) + cos(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1))) - sin(q6)*(sin(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - cos(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)))), - sin(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2))) - cos(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)), d4*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - lw*(sin(q5)*(sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + cos(al3)*sin(al2)*sin(q1)) - cos(q5)*(sin(q4)*(cos(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) - sin(al2)*sin(al3)*sin(q1)) - cos(q4)*(cos(q1)*cos(q2) - cos(al2)*sin(q1)*sin(q2)))) + q3*sin(al3)*(cos(q1)*sin(q2) + cos(al2)*cos(q2)*sin(q1)) + q3*cos(al3)*sin(al2)*sin(q1)]
        [ sin(q6)*(sin(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) + cos(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + sin(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2))), cos(q6)*(sin(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) + cos(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) - sin(q6)*(cos(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + sin(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2))),   sin(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) - cos(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2)), d4*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2)) - lw*(cos(q5)*(cos(q4)*(cos(q2)*sin(q1) + cos(al2)*cos(q1)*sin(q2)) - sin(q4)*(cos(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) + cos(q1)*sin(al2)*sin(al3))) + sin(q5)*(sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - cos(al3)*cos(q1)*sin(al2))) + q3*sin(al3)*(sin(q1)*sin(q2) - cos(al2)*cos(q1)*cos(q2)) - q3*cos(al3)*cos(q1)*sin(al2)]
        [                                                                                                                                                                     cos(q6)*(sin(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) + cos(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2))) - sin(q6)*(cos(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) - sin(al2)*sin(q2)*sin(q4)),                                                                                                                                                                   - cos(q6)*(cos(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) - sin(al2)*sin(q2)*sin(q4)) - sin(q6)*(sin(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) + cos(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2))),                                                                                                       sin(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2)) - cos(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)),                                                                                                                                                                             d4*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) - lw*(sin(q5)*(cos(al2)*cos(al3) - cos(q2)*sin(al2)*sin(al3)) + cos(q5)*(sin(q4)*(cos(al2)*sin(al3) + cos(al3)*cos(q2)*sin(al2)) + cos(q4)*sin(al2)*sin(q2))) + q3*cos(al2)*cos(al3) - q3*cos(q2)*sin(al2)*sin(al3)]
        [                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                                                                                                                                                                                                                                  1]];

    [sol_q, sol_count] = CalculateInvKin(transformation_eef , q_lb , q_ub , options);

    %check that the original seed is among the solutions
    found_sol = 0;
    for i=1:sol_count
        if norm(sol_q(i,:) - q_raw) < kTol
            found_sol = 1;
            break;
        end
    end

    if found_sol == 0
        disp('original joint angles were not found');
        str = sprintf(' %f ', q_raw);
        error('The seed was: %s', str);
    end
end

str = sprintf('tested %d poses with success', num_tests);
disp(str);