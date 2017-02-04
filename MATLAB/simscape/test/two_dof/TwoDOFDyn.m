% TwoDOFDyn.m
% Lee Magnusson
% 1/26/17

% inverse dynamics for 2 link arm for validation
% eq 4.11 mls94-manipdyn_v1_2

classdef TwoDOFDyn
    properties
        l1 = 1;
        l2 = 1;
        l1c = .4;
        l2c = .6;
        m1 = .5;
        m2 = .25;  
    end
    methods 
        function tau = invDyn(o,q,qd,qdd)
            tau1 = o.m1*o.l1c^2*qdd(:,1) + o.m2*(o.l1^2*qdd(:,1) + o.l2c^2*(qdd(:,1) + qdd(:,2)) + ...
                            o.l1*o.l2c*((2*qdd(:,1) + qdd(:,2)).*cos(q(:,2)) - ...
                            (2*qd(:,1) + qd(:,2)).*qd(:,2).*sin(q(:,2))));
           
            tau2 = o.m2*(o.l2c^2*(qdd(:,1) + qdd(:,2)) + ...
                       o.l1*o.l2c*(qdd(:,1).*cos(q(:,2)) + ...
                       qd(:,1).^2.*sin(q(:,2))));
                   
            tau = [tau1,tau2];
        end
        
        function dy = forwardDyn(o,t,y,t_tau,v_tau)
            dy = zeros(4,1);
            q = y(3:4);
            qd = y(1:2);

            tau = interp1(t_tau,v_tau,t,'spline');

            M = [o.m1*o.l1c^2 + o.m2*(o.l1^2 + o.l2c^2 + o.l1*o.l2c*2*cos(q(2))),   o.m2*(o.l2c^2 + o.l1*o.l2c*cos(q(2)));
                 o.m2*(o.l2c^2 + o.l1*o.l2c*cos(q(2))),                             o.m2*o.l2c^2];

            F = [tau(1) + o.m2*o.l1*o.l2c*(2*qd(1)+qd(2))*qd(2)*sin(q(2));
                 tau(2) - o.m2*o.l1*o.l2c*qd(1)^2*sin(q(2))];

            dy(3:4) = qd;
            dy(1:2) = M\F;
        end
    end
end