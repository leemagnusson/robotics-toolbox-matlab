function [ Pm, Pt ] = motor_power_heat( Tj , qd, other_param)
%motor_power_heat calculates motor power

    motor_constant = other_param.motor_constant';
    
    motor_constant = repmat(motor_constant,size(Tj,1),1);
    Tmideal = Tj/100;
    qm = qd*100;
    Tm = zeros(size(Tj));
    for i = 1:size(Tj,2)
        Tm(:,i) =   CSD_Tin(qd(:,i),25,Tj(:,i),other_param.transmission{i});
    end
    
    % i^2r loss in motor
    Pm = (Tm./motor_constant).^2;
    
    % power loss in transmission
    Pt = abs(Tmideal-Tm).*abs(qm);
end

