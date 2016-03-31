% analyze_modes.m
% Lee Magnusson
% 2/8/16

% take a robot model, joint position and joint stiffness and analyze the modes of the
% system

% Use x_dot = A*x, frequencies = eigenvalues(A), modes = eigenvectors(A)

% I*q_ddot + C*q_dot + K*q = 0
% state = [q;q_dot], A = [zeros,eye;-I\K,-I\C]

r = r2;
r2.links(end).I(end) = 9e-7;  % trying to deal with poor scaling of the matrix
r2.links(end-3).I = r2.links(end).I*.5;

q = zeros(1,r.n);
qd = q;
%q(4*7) = -1;
%q(4*8) = .1;

Kideal = diag([27000,177000,177000,177000,...
               13000,136000,136000,136000,...
               13000,136000,136000,136000,...
               13000,136000,136000,136000,...
               13000,28000,28000,28000,...
               13000,28000,28000,28000,...
               13000,28000,28000,28000,...
               2e7,2e7,2e7,2e7,...
               60,60,60,60]);
Kmeas = diag([10900,6500,6500,1e6,...
              12100,6500,6500,1e6,...
              12100,5900,5900,1e6,...
              8400,4700,4700,1e6,...
              5700,2900,2900,1e6,...
              6500,3200,3200,1e6,...
              3000,1800,1800,1e6,...
              2e7,2e7,2e7,2e7,...
              200,200,200,200]);
K = diag(20000*[2,.5,1,.25,.5,.25,.25,1000,3e-3]);
K = Kmeas;

if ((r.n == 36) && (length(K) == 9))
    Kd = reshape(repmat(diag(K)',4,1),1,[]);
    Kd(34:35) = 100;
    
    K = diag(Kd);
end
tic
I = r.inertia(q);
toc
%C = r.coriolis(q,qd);
toc
zeta = .2*ones(r.n,1);
B = diag(zeta.*2.*sqrt(diag(I).*diag(K)));
%B = zeros(size(I));
%B = diag(diag(real(2*sqrt(I*K))));
%B = K*1e-5;
 C = zeros(size(I));
A = [zeros(length(q)),eye(length(q));
     -I\K, -I\B];

A2 = -I\K;
[V2,D2] = eig(A2);
freq2 = sqrt(abs(diag(D2)))/2/pi
V2

[V,D] = eig(A);
toc  
freq_natural = abs(diag(D))/2/pi;
freq_natural = freq_natural(1:2:end)
freq_damped = abs(imag(diag(D)))/2/pi;
freq_damped = freq_damped(1:2:end)
modes = V;
modes = modes(1:r.n,1:2:end);
modes = sign(imag(modes)).*abs(modes);
% normalize to help look at  
modes = modes/max(max(abs(modes)))


B = zeros(2*r.n,1);
B(1) = 1;
B(5) = 1;
B(13) = 1;
sys = ss(A,B,[eye(r.n),zeros(r.n)],0);
sysp = prescale(sys,{2*pi,200*pi});
figure(1);
h = bodeplot(sysp);
h.PhaseVisible = 'off';

w = logspace(1,4,20000);

[mag,~,wout] = bode(sysp,w);

figure(2);
semilogx(wout/2/pi,20*log10(squeeze(mag)));
xlabel('frequency (Hz)')
ylabel('magnitude (dB)')
grid on;
print -dpng modes/frequency.png

figure(3);
[y,t] = step(sysp,4);
%sum the duplicated axes and look at just joints
y2 = y(:,1:4:end)+y(:,4:4:end);
plot(t,y);

figure(4);
plot(t,y2);
grid on;
ylabel('joint angle (rad, m)');
xlabel('time (s)');
print -dpng modes/step.png

y3 = [y2,y(:,2:4:end),y(:,3:4:end)];

figure(5);
plot(t,y3);

% convert to cartesian 
cart = r2.jacob0(q)*diag(modes(:,end-4));
diff_motion_abs = sqrt(cart(1,:).^2 + cart(2,:).^2 + cart(3,:).^2);

array2table(reshape(modes(:,end-5),4,[]),'rownames',{'actuator axis','vaxis1','vaxis2','vaxis3'},'variablenames',{'j1','j2','j3','j4','j5','j6','j7','j8','j9'})

%% movie
i_mode = 36-5;
fname = 'mode2';

movie_mode = modes(:,i_mode);
f_mode = freq_damped(i_mode);
t = (0:1/60:1-1/60)';
qmode = .1*sin(2*pi*t)*movie_mode'+repmat(q,length(t),1);

r2.plot(qmode,'workspace',ws,'movie','movie')
system(['ffmpeg -r 60 -i movie/%04d.png -c:v libx264 -preset slow -tune animation -crf 22 ', fname,'.mkv']);