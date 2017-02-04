function freq = fftfreq(n,dt)

if (mod(n,2) == 0)
    freq = [0:n/2-1,-n/2:-1]/dt/n;
else
    freq = [0:(n-1)/2,-(n-1)/2:-1]/dt/n; 
    
end