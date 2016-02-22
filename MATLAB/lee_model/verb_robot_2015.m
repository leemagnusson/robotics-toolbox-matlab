% verb_robot2015.m
% Lee Magnusson
% 12/23/15

% verb_robot2016.m
% Lee Magnusson
% 2/1/16

close all;

global r other_param

robot_dir = 'robot 2015'

baseT = r2t(roty(-pi/2));
[r,other_param] = create_robot(robot_dir,baseT)

r.links(8).qlim = [-.1 .1];
r.links(9).G = 0;

initial_robot_tests(r,other_param,robot_dir);


% % add translation
% close all;
% 
% clear q0 r namemap
% global q0 r namemap
% 
% namemap = {'l1','arm1_shoulder_pitch_link.txt','CSD25_100';
%            'l2','arm1_shoulder_roll_link.txt','CSD20_100';
%            'l3','arm1_elbow_link.txt','CSD20_100';
%            'l4','arm1_forearm_roll_link.txt','CSD20_100';
%            'l5','arm1_spherical_base_link.txt','CSD14_100';
%            'l6','arm1_spherical_roll_link.txt','CSD14_100';
%            'l7','tool_driver.txt','CSD14_100';
%            'l8','tool_driver.txt','CSD14_100';      %todo mass and transmission properties for these
%            'l9','tool_driver.txt','CSD14_100'};
% transmission_efficiency = .6*ones(1,length(namemap));
% 
% offsets = pi/180*[(90-15.13);0;0;0;-25;-30.22;90-88.51];
%        
% l1 = Revolute('a',0,'d',0,'alpha',pi/2,'offset',offsets(1));
% l2 = Revolute('a',.02,'d',.32485,'alpha',-pi/2,'offset',offsets(2));
% l3 = Revolute('a',.0254,'d',-.0211,'alpha',-pi/2,'offset',-pi/2+offsets(3));
% l4 = Revolute('d',.4153,'alpha',-pi/2,'offset',offsets(4));
% l5 = Revolute('d',-.00508,'alpha',pi/2,'offset',offsets(5));
% l6 = Revolute('d',.4318,'alpha',-(90-69.59)*pi/180-pi/2,'offset',offsets(6));
% l7 = Revolute('offset',pi/2+offsets(7),'a',0,'alpha',-100*pi/180);
% l8 = Prismatic('offset',0);  % note needed to change pjoint to link ni animate to get this to work for plot3d
% l9 = Revolute('d',.025);
% l = [l1,l2,l3,l4,l5,l6,l7,l8,l9];
% 
% % import mass properties from solidworks files
% % use solidworks evalute mass properties, select the coordinate system,
% % and copy and paste to a text file
% for i = 1:size(namemap,1)
%     link = namemap{i,1};
%     fname = namemap{i,2};
%     trans = namemap{i,3};
%     l(i) = link_add_mass_properties(l(i), ['mass properties 2015/', fname],trans);
%     % comment out below to use motor inertia
%     %l(i).Jm = 0;
% end
% 
% % temporary until get mass properties
% l(8).m = 0;
% l(8).I = zeros(3);
% l(9).m = 0;
% l(9).I = zeros(3);
% l(9).G = 0;
% 
% 
% 
% baseT = r2t(roty(-pi/2));
% gravity = baseT*[0,0,-9.81,1]';
% 
% r = SerialLink(l,'base',baseT);
% r.gravity = gravity(1:3);
% 
% r
% 
% q0 =  zeros(size(l));
% 
% %q0(8) = .2
% qmax = [pi/2+20*pi/180, 0, -20*pi/180, 0, 0, 0, -67.7*pi/180];
% qside1 = [0,pi/2,0,0,0,0,-67.7*pi/180];
% qside2 = [0,pi/2,0,0,-pi/2,0,-67.7*pi/180];
% 
% qtest = {q0,qmax,qside1,qside2};
% qtest = {q0}
% 
% for i = 1:length(qtest)
%     q = qtest{i};
%     
%     figure(1);
%     r.plot(q,'workspace',[-1,1,-1,1,-.1,1]);
% 
%     figure(2);
%     r.plot3d(q,'path','robot stl 2015','workspace',[-1,1,-.5,1.25,-.1,1]);
%     
%     % lines below for exporting 3dpdf
% %     hfloor = findobj(gcf,'type','surface');
% %     delete(hfloor);
% %     %h = findobj(gcf,'type','patch');
% %     group = findobj(gca,'Tag',r.name);
% %     h = group.UserData;
% %     for i = 2:8
% %         new_vert = apply_transform(h.link(i).Matrix,h.link(i).Children(1).Vertices);
% %         h.link(i).Children(1).Vertices = new_vert;
% %         h.link(i).Children(1).EdgeColor = 'none';
% %     end
%     %set(h,'FaceLighting','gouraud');
% 
%     Tj = r.gravload(q)
%     
%     Pm = motor_power_heat(Tj,r,transmission_efficiency);
%     Pm_sum = sum(Pm)
% end
% 
% %set(gcf,'Renderer','painters')
% %print -deps2c images/model
