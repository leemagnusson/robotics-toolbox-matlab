% robot_movie.m

function robot_movie(r,q,ws,fps,fname,threed)

if ~exist('fname','var')
    fname = 'movie';
end
if ~exist('threed','var')
    threed = true;
end
if ~exist('fps','var')
    fps = 60;
end

figure()

try
    rmdir movie s
end

r.plot3d(q,'workspace',ws,'movie','movie');
system(['ffmpeg -r ' num2str(fps) ' -i movie/%04d.png -c:v libx264 -preset slow -tune animation -crf 22 ', fname,'.mkv']);