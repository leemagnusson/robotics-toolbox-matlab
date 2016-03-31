

tmax = 3;
[t,q,qd] = r.fdyn(tmax,@(a,b,c,d) zeros(1,r.n), q0);

if ~exist('tests','dir')
    mkdir tests
end
robot_movie(r,t,q,'fname','tests/gravity','savefile',true);