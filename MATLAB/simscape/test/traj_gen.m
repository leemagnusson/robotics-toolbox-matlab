% traj_gen.m
% Lee Magnusson
% 1/26/16

% generate a trajectory for robot simulation
% f_max, max frequency content
% n, number of signals

function [q,qdot,qddot] = traj_gen(t, f_max, n)

% random data in frequency domain up to f_max,
% symmetric for all real output
f = zeros(size(t));
dt = t(2)-t(1);

i = floor(f_max*dt*length(t));


for j = 1:n
    f_dat = rand(i,1)-.5;
    i_dat = 1i*(rand(i,1)-.5);
    f(1:i) = f_dat + i_dat;
    f(end-i+2:end) = flipud(f_dat(2:end)-i_dat(2:end));
    %f = f-mean(f);
    f(1) = 0;

    omega = fftfreq(length(t),dt)'*2*pi;

    qddot(:,j) = real(fft(f));
    fi = f./(1i*omega);
    fi(1) = 0;
    qdot(:,j) = real(fft(-fi)); 
    fii = f./(-omega.^2);
    fii(1) = 0;
    q(:,j) = real(fft(fii));
end