% start by analyzing ideal braking distance
m = get_motors('cell');

V = 48;
m = {m{3},m{2},m{2},m{1},m{1},m{1},m{1},m{1},m{1},m{1},m{1}};
for i = 1:length(m);
    vel = V*m{i}.kv;
    b(i) = m{i}.km^2/m{i}.R;
    tau = m{i}.J/b(i);
    Ts = 3*tau;
    th = m{i}.J*vel/b(i);
    Ti = b(i)*vel;
    Tmax(i) = m{i}.Tmax;
end



% Get a bunch of q positions
q_list = GetPositionArray(r,u);%q0;

% Set tool position to end
r.tool(3,4) = .28;  % m
r.links(10).m = 2; % kg

v_i = .09; % m/s initial velocity
t_stop = .1875; % s
sat = @(x,lim_min,lim_max) min(max(x,lim_min),lim_max);

%%
t_p = zeros(size(q_list));
t_g = zeros(size(q_list));
Tm_dyn_p = zeros(size(q_list));
parfor i = 1:size(q_list,1)
    if (mod(i,1000)==0)
        disp(i);
    end
    q = q_list(i,:);
    t_p_tmp = JointImpulse(r,q,v_i)';
    t_g_tmp = r.gravload(q);

    qd = zeros(size(q));
    [t,qs,qds] = r.fdyn(t_stop,@(r,t,q,qd) t_g_tmp+t_p_tmp/t_stop, q, qd);
    xs = r.fkine(q) - r.fkine(qs(end,:));
    xds = r.jacob0(q)*qds(end,:)';
 
    Tm_dyn = sat(repmat(b*100^2,size(qds,1),1).*qds,-repmat(Tmax*100,size(qds,1),1),repmat(Tmax*100,size(qds,1),1));
    Tm_dyn_p(i,:) = trapz(t,Tm_dyn);  % motor impulse due to braking

    t_p(i,:) = t_p_tmp;
    t_g(i,:) = t_g_tmp;
end
%%
[t_p_max,i_impulse_max] = max(abs(t_p),[],1);
t_impulse_max = t_p_max/t_stop
i_impulse_max = sub2ind(size(t_p),i_impulse_max,1:11);

abs(Tm_dyn_p(i_impulse_max))

t_impulse = t_p/t_stop;
[t_total_max,i_total_max] = max(abs(t_impulse)+abs(t_g),[],1)

i_total_max = sub2ind(size(t_p),i_total_max,1:11);
impulse_total = abs(t_p(i_total_max)) + abs(t_g(i_total_max))*t_stop

[E_max,i_E_max] = max(.5*q_list.*t_p,[],1);

%%
% meaningless
%impulse_braking = abs(Tm_dyn_p(i_total_max))