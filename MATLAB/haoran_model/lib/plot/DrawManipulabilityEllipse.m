function DrawManipulabilityEllipse(sigma_matrix,u_matrix,p_eef,ellipse_size)
hold on
sigma_matrix = sigma_matrix.^(1/2.5);
scale_axis = ellipse_size / sigma_matrix(1,1);
xr = sigma_matrix(1,1) * scale_axis;
yr = sigma_matrix(2,2) * scale_axis;
zr = sigma_matrix(3,3) * scale_axis;
[x,y,z] = ellipsoid(0,0,0,xr,yr,zr,20);
for index1 = 1 : length(x)
    for index2 = 1 : length(x)
        temp = [x(index1,index2);y(index1,index2);z(index1,index2)];
        temp = u_matrix * temp + p_eef;
        x(index1,index2) = temp (1);
        y(index1,index2) = temp (2);
        z(index1,index2) = temp (3);
    end
end

surf(x,y,z,'FaceColor',[1 0 0],'FaceAlpha',0.2,'EdgeColor','none');
hold on
DrawCoordinateSystem([0 0 ellipse_size/2],u_matrix,p_eef,'rgb')
end
