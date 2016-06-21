function [ Tout ] = CalculateTF( T, i_0 , i_f)
%Calculates the transformation T from start to end frame 
Tout = sym(eye(4));

if i_0 > i_f
    Tout = CalculateInvT(CalculateTF(T ,i_f , i_0));
else
    for i=i_0+1:i_f
        Tout = Tout*T(:,:,i);
    end
end
end

