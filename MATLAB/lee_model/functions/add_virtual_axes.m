% Lee Magnusson
% add 3 virtual axes for every joint in a robot model such that we can
% model orthogonal stifnesses. 
% 2/9/16

close all;
n = length(r.links());

v = 3
for i = 1:n
    links2(v*(i-1)+v) = r.links(i);
%    links2(4*(i-1)+2) = Revolute('offset',0,'alpha',0);
%    links2(4*(i-1)+3) = Revolute('offset',0,'alpha',0);
%    links2(4*(i-1)+4) = Revolute('offset',0,'alpha',0);
    links2(v*(i-1)+1) = Revolute('offset',pi/2,'alpha',pi/2);
    links2(v*(i-1)+2) = Revolute('offset',pi/2,'alpha',pi/2);
%    links2(v*(i-1)+3) = Revolute('offset',pi/2,'alpha',pi/2);
end

r2 = SerialLink(links2)
r2.base = r.base
r2.gravity = r.gravity

q = zeros(1,length(links2));
figure(1);
r2.plot(q,'workspace',[-1 1 -1 1 -1 1],'jaxes');