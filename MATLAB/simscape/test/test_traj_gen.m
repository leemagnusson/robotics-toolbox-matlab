% test

t = linspace(0,10,512*8)';
dt = t(2)-t(1);

rng(0)

f_max = [.5,1,5,10];
n = 1;

for i = 1:length(f_max)
    [q,qd,qdd] = traj_gen(t, f_max(i),1);

    qdt = diff(q)./diff(t);
    qddt = diff(qd)./diff(t);
    qddtt = diff(q,2)/dt^2;

    assert(abs(mean(q)) < 10*eps);
    assert(abs(mean(qd)) < 10*eps);
    assert(abs(mean(qdd)) < 10*eps);

    % frequency content less than fmax
    f = fft(qdd);
    assert(all(abs(f(abs(fftfreq(length(t),dt)) > f_max(i))) < 100000*eps))  % shouldn't need to be that tolerant

    % derivatives similar to time derivative
    assert(mean(abs(qd(2:end)-qdt))/length(t) < 1e-3)   % nor should these
    assert(mean(abs(qdd(2:end)-qddt))/length(t) < 1e-3)
    assert(mean(abs(qdd(2:end-1)-qddtt))/length(t) < 1e-3)
end