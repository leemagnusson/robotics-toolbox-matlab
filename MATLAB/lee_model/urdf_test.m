
urdf_dir = 'V1_Arm_URDF';
u = URDF([urdf_dir '/V1_Arm_URDF.URDF']);
stl_dir = [urdf_dir '/asciistl'];
mkdir(stl_dir);

% main conversion
[dh,T_mod,T] = joints2dh(u.joints(1:12));
r = SerialLink([dh(2:end-1,:);0,0,0,0,0,0],'base',T(:,:,1));
r.gravity = t2r(T(:,:,1))*r.gravity;

q0 = zeros(1,r.n);

% convert stl binary to ascii, and link%i format expected by toolbox
for i = 1:r.n
    if isfield(u.links{i},'visual')
        fname = u.links{i}.visual.geometry.mesh.filename;
        fname = fname(11:end);
        [v, f, n, name] = stlRead(fname);
        vnew = [v';ones(1,length(v))];
        if (i~=1)
           vnew = inv(T(:,:,i))*inv(T_mod(:,:,i-1))*vnew;
        end
        stlWrite([urdf_dir '/asciistl/link' num2str(i-1) '.stl'],f,vnew(1:3,:)','mode','ascii');
    end
end

% todo double check these transformations
for i = 2:r.n+1
    if isfield(u.links{i},'inertial')
        r.links(i-1).m = u.links{i}.inertial.mass.value;
        Tinertia = inv(T(:,:,i))*inv(T_mod(:,:,i-1));
        rnew = Tinertia*[u.links{i}.inertial.origin.xyz';1];
        r.links(i-1).r = rnew(1:3);
        Tinertia2 = rpyxyz2tr(u.links{i}.inertial.origin.rpy, ...
                              u.links{i}.inertial.origin.xyz);
        Tinertia3 = Tinertia2*Tinertia;
        I6 = u.links{i}.inertial.inertia;
        I = [I6.ixx, I6.ixy, I6.ixz;
             I6.ixy, I6.iyy, I6.iyz;
             I6.ixz, I6.iyz, I6.izz];
        r.links(i-1).I = t2r(Tinertia3)*I*t2r(Tinertia3)';
    end
end
    
figure(1);clf;
r.plot(q0,'jaxes');

figure(2); clf;
r.plot3d(q0,'path',stl_dir);

%r.gravload(q0)