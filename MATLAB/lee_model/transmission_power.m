function [ Pout, Pin, Ploss ] = transmission_power( tau,qd,transmission_efficiency )
%transmission_power calculates loss in transmission

    transmission_efficiency = .6;
    output_power = tau.*qd;
    input_power = zeros(size(output_power));
    input_power(output_power>0) = output_power(output_power>0)*(1+transmission_efficiency);
    input_power(output_power<0) = output_power(output_power<0)*(transmission_efficiency);

    Pout = output_power;
    Pin = input_power;
    Ploss = input_power-output_power;

end

