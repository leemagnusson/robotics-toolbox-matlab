function r_out = add_base_axes(r)

base = SerialLink([0,0,0,pi/2,0,pi/2;
                   0,0,0,0,1,0;
                   0,0,0,pi/2,0,pi/2;
                   0,0,0,0,1,0;
                   0,0,0,pi/2,0,pi/2;
                   0,0,0,0,1,0;]);

r_out = SerialLink([base.links,r.links],'base',r.base);
r_out.gravity = r.gravity;


