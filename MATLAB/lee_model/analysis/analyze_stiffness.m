% analyze_stiffness.m
% Lee Magnusson 4/4/16
% Look at joint stiffness contributions to end effector

q = q0;

% todo put dataload into function
v1_data = loadjson([lee_model_dir filesep 'data/v1_other_data.json']);
hd_data = loadjson([lee_model_dir filesep 'data/harmonic_drive.json']);

kdiag = [];
for i=1:r.n
    transmission = v1_data{i}.transmission;
    kdiag(i) = hd_data.(transmission).k;
end

% test 10x less stiff on pitch joints
kdiag(10:11) = kdiag(10:11)/10;
% test out at .28
r.tool(3,4) = -.38;

J = r.jacob0(q);
M = r.inertia(q);

% % solve for a predefined stiffness matrix
% Kx = [1e5*ones(3,1);1e4*ones(3,1)];
% K = J'*diag(Kx)*J;
% % test with K solved for
% kdiag = diag(K);
% 
% % more advanced solve
% A = [];
% for i = 1:r.n
%     A(:,i) = J(:,i)*J(:,i)'*ones(6,1);
% end
% % A*x = b
% kdiag = A\Kx;
% % doesn't seem to work well
% 
% % try place
% wdes = 20*2*pi*ones(r.n,1);
% A = zeros(r.n);
% K = -place(A,inv(M),wdes.^2);
% 
% kdiag = diag(K);
% kdiag = Kmax;

% Cx is compliance matrix at end effector in cartesian space
Cx = zeros(6,6,r.n);
% these relate to 4 quadrants of matrix
Cxf = zeros(1,r.n);
Cxtau = zeros(1,r.n);
Cthf = zeros(1,r.n);
Cthtau = zeros(1,r.n);
for i = 1:r.n
    Cx(:,:,i) = J(:,i)*J(:,i)'/kdiag(i);
    Cxf(i) = norm(Cx(1:3,1:3,i));
    Cxtau(i) = norm(Cx(1:3,4:6,i));
    Cthf(i) = norm(Cx(4:6,1:3,i));
    Cthtau(i) = norm(Cx(4:6,4:6,i));
end

% Cthtau turns out to be just the stiffness of the joint itself
% Cxtau and Cthf tau turn out to equal each other but are relevant

% stiffness with respect to base joint
kxf_base = Cxf(1)./Cxf
kxtau_base = Cxtau(1)./Cxtau


% look at effect of each joint on lowest resonant frequency
% ignoring damping
[v,d] = eig(M\diag(kdiag));
wbase = 1/2/pi*sqrt(diag(d));
% find the mode that each joint contributes the most to
[~,maxi] = max(abs(v),[],2);
% Also find the lowest 3 frequencies
n_freq = 3;
[~,tmpi] = sort(wbase);
lowi = tmpi(1:n_freq);

wtest=[];
freq_change=[];
for i = 1:r.n
    Ktest = diag(kdiag);
    Ktest(i,i) = Ktest(i,i)*1.1;    % see result of 10% increase in stiffness
    [vtest(:,:,i),dtest] = eig(r.inertia(q)\Ktest);
    wtest(:,i) = 1/2/pi*sqrt(diag(dtest));
    [~,maxtesti(i)] = max(abs(vtest(i,:,i)),[],2);
    wtest_max(i) = wtest(maxtesti(i),i);
    freq_change(i) = (wtest_max(i)-wbase(maxi(i)))/wbase(maxi(i));
    [~,tmpi] = sort(wtest(:,i));
    lowtesti = tmpi(1:n_freq);
    freq_change_lowest(:,i) = (wtest(lowtesti,i)-wbase(lowi))./wbase(lowi);
    
end
% make clear non changes in lowest frequency
freq_change_lowest(freq_change_lowest<1e-4) = 0;

wbase(maxi)'
freq_change_lowest*100