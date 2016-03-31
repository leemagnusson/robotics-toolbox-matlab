%#eml
function R = RotationMatrix_rad(angle,axis)
    if axis(1) == 1
        R = [1 0 0
             0 cos(angle) -sin(angle)
             0 sin(angle) cos(angle)];
    elseif axis(2) == 1
        R = [cos(angle) 0 sin(angle)
             0 1 0
             -sin(angle) 0 cos(angle)];
    else
        R = [cos(angle) -sin(angle) 0
             sin(angle) cos(angle) 0 
             0 0 1];
    end
    
end