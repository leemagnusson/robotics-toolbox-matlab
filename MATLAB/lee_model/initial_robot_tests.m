function initial_robot_tests(r,other_param, robot_dir)

global q0


q0 =  zeros(size(r.links));


qmax = [pi/2+20*pi/180, 0, -20*pi/180, 0, 0, 0, -67.7*pi/180,0,0];
qside1 = [0,pi/2,0,0,0,0,-67.7*pi/180,0,0];
qside2 = [0,pi/2,0,0,-pi/2,0,-67.7*pi/180,0,0];
q1 = [.2,1,.1,.2,.1,.2,1,.1,1];

qtest = {q0,qmax,qside1,qside2,q1};
qtest = {q0};

for i = 1:length(qtest)
    q = qtest{i};
    
    figure(1);
    r.plot(q,'workspace',[-1,1,-1,1,-.1,1],'jaxes');

    figure(2);
    r.plot3d(q,'path',[robot_dir '/stl'],'workspace',[-1,1,-.5,1.25,-.1,1],'jaxes');
    
    % lines below for exporting 3dpdf
%     hfloor = findobj(gcf,'type','surface');
%     delete(hfloor);
%     %h = findobj(gcf,'type','patch');
%     group = findobj(gca,'Tag',r.name);
%     h = group.UserData;
%     for i = 2:8
%         new_vert = apply_transform(h.link(i).Matrix,h.link(i).Children(1).Vertices);
%         h.link(i).Children(1).Vertices = new_vert;
%         h.link(i).Children(1).EdgeColor = 'none';
%     end
    %set(h,'FaceLighting','gouraud');

    Tj = r.gravload(q)
    
    Pm = motor_power_heat(Tj,.01*sign(Tj),other_param);
    Pm_sum = sum(Pm)
    
    
end

%set(gcf,'Renderer','painters')
%print -deps2c images/model