
close all
rtmp = SerialLink([0,0,0,pi/2,0,pi/2;
                   0,0,0,pi/2,0,pi/2;
                   0,0,0,pi/2,0,pi/2])

q = zeros(1,length(rtmp.links))
figure(1);
rtmp.plot(q,'workspace',[-1 1 -1 1 -1 1],'jaxes')