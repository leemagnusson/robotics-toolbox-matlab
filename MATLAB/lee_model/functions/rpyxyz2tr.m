function tr = rpyxyz2tr(rpy,xyz);

tr = r2t(rotz(rpy(3))*roty(rpy(2))*rotx(rpy(1)));
tr(1:3,4) = xyz;