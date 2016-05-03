function [ Pm, Pt, Tm, Im ] = motor_torque_speed_pwr( Tj , qd, other_param)
%motor_torque_speed_pwr Estimates motor power for a given joint torque and
%speed.

    motor_constant = other_param.motor_constant';
    
    motor_constant = repmat(motor_constant,size(Tj,1),1);
    % Ideal motor torque (gear ratio = 100)
    Tmideal = Tj/100;
    qm = qd*100;
    Tm = zeros(size(Tj));
    for i = 1:size(Tj,2)
        Tm(:,i) =   CSD_Tin(qd(:,i),25,Tj(:,i),other_param.transmission{i});
    end
    
    motor_coeff = zeros(size(Tj,2), 6);
    for i = 1:size(Tj,2)
        motor_coeff(i,:) = eval(other_param.mot_coeff{i});
    end
    
    %clip to motor's peak torque (can't go above that even if we try)
    Tm_clipped = Tm;
    for i = 1:size(Tm,2)
        Tm_col = Tm_clipped(:, i);
        Tm_col(Tm_col > other_param.mot_peak_torque(i)) = other_param.mot_peak_torque(i);
        Tm_col(Tm_col < -other_param.mot_peak_torque(i)) = -other_param.mot_peak_torque(i);
        Tm_clipped(:,i) = Tm_col;
    end
    
    
    % Current for motor torque and speed (assuming clipped torque)
    Im = (Tm_clipped + repmat(motor_coeff(:,2),1,size(Tj,1))' + repmat(motor_coeff(:,3),1,size(Tj,1))'.*qd)./repmat(motor_coeff(:,1),1,size(Tj,1))';
    
    % Power for motor current and speed:
    
    Pm = repmat(motor_coeff(:,6),1,size(Tj,1))' + repmat(motor_coeff(:,5),1,size(Tj,1))'.*Im + repmat(motor_coeff(:,4),1,size(Tj,1))'.*Im.^2 + repmat(motor_coeff(:,2),1,size(Tj,1))'.*qd + repmat(motor_coeff(:,3),1,size(Tj,1))'.*qd.^2;
    
    % power loss in transmission
    Pt = abs(Tmideal-Tm).*abs(qm);
end

