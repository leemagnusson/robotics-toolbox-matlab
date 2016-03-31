% gravity_comp_demo.m
% Lee Magnusson 3/28/16
% simulate gravity comp with some forces applied with joint torque sensors
% and some damping applied

% Tapplied = Tgravity + J'*F - b*th_dot
%%
tmax = 4;
forces = loadjson('gravity_comp_demo_forces.json');

Txd = r.fkine(q0);
xd = transl(Txd);
xd = [transl(Txd);tr2rpy(Txd)'];
[t,q,qd] = r.fdyn(tmax,@(r,t,q,qd) gravity_comp_fun(r,t,q,qd,forces,xd),q0);

figure(1);
[ti,qi] = robot_movie(r,t,q);

%%

mkdir movie
for i = 1:length(ti)
    [~,Tall] = r.fkine(qi(i,:));
    r.plot3d(qi(i,:)); hold on;
    for j = 1:length(forces)
        if(ti(i) > forces{j}.tmin && ...
            ti(i) < forces{j}.tmax)
            P = transl(Tall(:,:,forces{j}.joint-1));    % maybe need -1
            F = .2*forces{j}.F/norm(forces{j}.F);
            quiver3(P(1),P(2),P(3),F(1),F(2),F(3),'LineWidth',2,'MaxHeadSize',1);
        end
    end
    [xs,ys,zs] = sphere();
    sph_sz = .03;
    surf(sph_sz*xs+xd(1), sph_sz*ys+xd(2), sph_sz*zs+xd(3));
    a = uicontrol('style','text','string','t = 0','units','normalized','position',[0,0,.1,.1]);
    a.String = ['t = ' num2str(ti(i),2)];
    view(115,25);
    drawnow
    print('-dpng',sprintf('movie/%04d',i));
end

%%
fname = 'gravity_comp'
system(['ffmpeg -r 60 -i movie/%04d.png -c:v libx264 -preset slow -tune animation -crf 22 ', fname,'.mkv']);
rmdir movie s