        
t = @(rotation,expected) assert(any(any(abs(rotation-expected)<2*eps)));

theta = linspace(-10,10,100);
rng(0);
vrand = rand(3,10); % random rotation vectors

for i = 1:length(theta)
    th = theta(i);
    rotation = RotationAxisAngle([0,0,1],th);
    expected = [cos(th) -sin(th) 0;
                sin(th) cos(th) 0;
                0 0 1];
    t(rotation,expected)
    
    rotation = RotationAxisAngle([1,0,0],th);
    expected = [1 0 0;
                0 cos(th) -sin(th);
                0 sin(th) cos(th)];
    t(rotation,expected)
    
    for j = 1:size(vrand,2)
        % rodrigues rotation formula
        % https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
        k = vrand(:,j)./norm(vrand(:,j));
        e1 = [1;0;0]; e2 = [0;1;0]; e3 = [0;0;1];
        v1 = e1*cos(th) + cross(k,e1)*sin(th) + k*(k'*e1)*(1-cos(th));
        v2 = e2*cos(th) + cross(k,e2)*sin(th) + k*(k'*e2)*(1-cos(th));
        v3 = e3*cos(th) + cross(k,e3)*sin(th) + k*(k'*e3)*(1-cos(th));
        
        rotation = RotationAxisAngle(vrand(:,j),th);
        t(rotation,[v1,v2,v3])
        
        % Haoran implementation
        axis = k;
        s = th*[0 -axis(3) axis(2);axis(3) 0 -axis(1);-axis(2) axis(1) 0];
        expected = expm(s);
        
        t(rotation,expected)
    end
end

