% robot_movie.m

function [t,q] = robot_movie(r,t,q,varargin)

fname = 'movie';
threed = true;
fps = 60;
interp = true;
ws = [-1 1 -1 1 -1 1];
figure_size = [];
savefile = false;


for i = 1:2:length(varargin)
    switch varargin{i}
        case 'fname'
            fname = varargin{i+1};
        case 'threed'
            threed = varargin{i+1};
        case 'fps'
            fps = varargin{i+1};
        case 'interp'
            interp = varargin{i+1};
        case 'workspace'
            ws = varargin{i+1};
        case 'size'
            figure_size = varargin{i+1};
        case 'savefile'
            savefile = varargin{i+1};
        otherwise
            error(['Unrecognized option: ' varargin{i}]);
    end
end

if (interp)
    ti = 0:1/fps:max(t);
    q = interp1(t,q,ti);
    t = ti;
end

if (savefile)
    close all
    % seems to save pictures from all open figs otherwise
end
h = gcf();
if ~isempty(figure_size)
    h.Position(3:4) = figure_size;
end

try
    rmdir movie s
end

% a = uicontrol('style','text','string','t = ','units','normalized','position',[0,0,.1,.1])
if (savefile)
    r.plot3d(q,'workspace',ws,'movie','movie');
    system(['ffmpeg -r ' num2str(fps) ' -i movie/%04d.png -c:v libx264 -preset slow -tune animation -crf 22 ', fname,'.mkv']);
else
    r.plot3d(q,'workspace',ws);
end
