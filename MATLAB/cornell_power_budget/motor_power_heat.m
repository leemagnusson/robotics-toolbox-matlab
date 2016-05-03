function [ Pm, Pt, Tm, Tm_clipped, Pm_clipped ] = motor_power_heat( Tj , qd, other_param)
%motor_power_heat calculates motor power

    motor_constant = other_param.motor_constant';
    
    motor_constant = repmat(motor_constant,size(Tj,1),1);
    % Ideal motor torque (gear ratio = 100)
    Tmideal = Tj/100;
    qm = qd*100;
    Tm = zeros(size(Tj));
    for i = 1:size(Tj,2)
        Tm(:,i) =   CSD_Tin(qd(:,i),25,Tj(:,i),other_param.transmission{i});
    end
    
    %clip to motor's peak torque (can't go above that even if we try)
    Tm_clipped = Tm;
    for i = 1:size(Tm,2)
        Tm_col = Tm_clipped(:, i);
        Tm_col(Tm_col > other_param.mot_peak_torque(i)) = other_param.mot_peak_torque(i);
        Tm_col(Tm_col < -other_param.mot_peak_torque(i)) = -other_param.mot_peak_torque(i);
        Tm_clipped(:,i) = Tm_col;
    end
    
    % i^2r loss in motor
    Pm = (Tm./motor_constant).^2;
    Pm_clipped = (Tm_clipped./motor_constant).^2;
    
    % power loss in transmission
    Pt = abs(Tmideal-Tm).*abs(qm);
end

