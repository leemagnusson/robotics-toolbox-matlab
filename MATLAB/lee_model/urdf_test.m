
urdf_dir = 'V1_Arm_URDF';
u = URDF([urdf_dir '/V1_Arm_URDF.URDF']);
stl_dir = [urdf_dir '/asciistl'];
mkdir(stl_dir);



[dh,T_mod,T] = rpyxyz2dh(u.joints(1:12))

baseT = rpyxyz2tr(u.joints{1}.origin.rpy,u.joints{1}.origin.xyz);
dh(1,:) = [0,0,0,1.57,0,1.06];
r = SerialLink([dh(2:11,:);0,0,0,0,0,0],'base',baseT);
q0 = zeros(1,r.n);

% convert stl binary to ascii, and link%i format expected by toolbox
for i = 2:10
    if isfield(u.links{i},'visual')
        fname = u.links{i}.visual.geometry.mesh.filename;
        fname = fname(11:end);
        [v, f, n, name] = stlRead(fname);
        vnew = [v';ones(1,length(v))];
        vnew = inv(T(:,:,i))*inv(T_mod(:,:,i-1))*vnew;
        stlWrite([urdf_dir '/asciistl/link' num2str(i-1) '.stl'],f,vnew(1:3,:)','mode','ascii');
    end
end


figure(1);clf;
r.plot(q0,'jaxes');

figure(2); clf;
r.plot3d(q0,'path',stl_dir);