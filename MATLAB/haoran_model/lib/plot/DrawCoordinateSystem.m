function DrawCoordinateSystem(axis_length,rotation,translation,color,name_frame)
% DrawCoordinateSystem(axis_length,rotation,translation,color,name_frame)
% this function draws a coordinate system with rgb color
% axis_length = 1; % x y z axis length, could be either a scalar or a 3 by 1 vector
% rotation = eye(3); % rotation matrix of the frame
% translation = [0;0;0]; % origin of frame
% color = 'rgb'; % rgb color
% name_frame = 'b' % base frame
% DrawCoordinateSystem(axis_length,rotation,translation,color,name_frame);
hold on;
if length(axis_length) == 1
    axis_length_modified = [axis_length;axis_length;axis_length];
elseif length(axis_length) == 3
    axis_length_modified = axis_length;
else
    error('wrong input axis length, either scalar or 3 by 1 vecotr')
end

x_hat = rotation(:,1);  % unit vectors
y_hat = rotation(:,2);
z_hat = rotation(:,3);

x_tip = translation + axis_length(1)*rotation(:,1); % axis tips
y_tip = translation + axis_length(2)*rotation(:,2);
z_tip = translation + axis_length(3)*rotation(:,3);

%drawing the axis lines
line([translation(1); x_tip(1)],[translation(2); x_tip(2)],[translation(3); x_tip(3)],'color',color(1),'linewidth',2);
line([translation(1); y_tip(1)],[translation(2); y_tip(2)],[translation(3); y_tip(3)],'color',color(2),'linewidth',2);
line([translation(1); z_tip(1)],[translation(2); z_tip(2)],[translation(3); z_tip(3)],'color',color(3),'linewidth',2);

% setting tip cone variables
cap_height = axis_length(1)/5;
cap_radius = axis_length(1)/10;
num_sample = 20;

[x_cone,y_cone,z_cone] = cylinder([cap_radius 0], num_sample);
z_cone = z_cone * cap_height;

%drawing the cone for x_axis
rotation_axis = rotation * RotationAxisAngle([0;1;0],pi/2);
for index = 1 : length(z_cone)
    point_clouds1 = rotation_axis * [x_cone(1,index);y_cone(1,index);z_cone(1,index)] + x_tip;
    x(1,index) = point_clouds1(1);
    y(1,index) = point_clouds1(2);
    z(1,index) = point_clouds1(3);
    point_clouds2 = rotation_axis * [x_cone(2,index);y_cone(2,index);z_cone(2,index)] + x_tip;
    x(2,index) = point_clouds2(1);
    y(2,index) = point_clouds2(2);
    z(2,index) = point_clouds2(3);
end
surf(x,y,z,'EdgeColor',color(1),'FaceColor',color(1),'EdgeLighting','phong','FaceLighting','phong');

%drawing the cone for y_axis
rotation_axis =  rotation * RotationAxisAngle([1;0;0],-pi/2);
for index = 1 : length(z_cone)
    point_clouds1 = rotation_axis * [x_cone(1,index);y_cone(1,index);z_cone(1,index)] + y_tip;
    x(1,index) = point_clouds1(1);
    y(1,index) = point_clouds1(2);
    z(1,index) = point_clouds1(3);
    point_clouds2 = rotation_axis * [x_cone(2,index);y_cone(2,index);z_cone(2,index)] + y_tip;
    x(2,index) = point_clouds2(1);
    y(2,index) = point_clouds2(2);
    z(2,index) = point_clouds2(3);
end
surf(x,y,z,'EdgeColor',color(2),'FaceColor',color(2),'EdgeLighting','phong','FaceLighting','phong');

%drawing the cone for z_axis
rotation_axis = rotation;
for index = 1 : length(z_cone)
    point_clouds1 = rotation_axis * [x_cone(1,index);y_cone(1,index);z_cone(1,index)] + z_tip;
    x(1,index) = point_clouds1(1);
    y(1,index) = point_clouds1(2);
    z(1,index) = point_clouds1(3);
    point_clouds2 = rotation_axis * [x_cone(2,index);y_cone(2,index);z_cone(2,index)] + z_tip;
    x(2,index) = point_clouds2(1);
    y(2,index) = point_clouds2(2);
    z(2,index) = point_clouds2(3);
end
surf(x,y,z,'EdgeColor',color(3),'FaceColor',color(3),'EdgeLighting','phong','FaceLighting','phong');

if nargin > 4, %add x y z label
   x_label = x_tip + 2 * cap_height*x_hat; 
   y_label = y_tip + 2 * cap_height*y_hat;
   z_label = z_tip + 2 * cap_height*z_hat;

   text(x_label(1),x_label(2),x_label(3),strcat('X_{',name_frame,'}'),'FontWeight','bold','color','k');
   text(y_label(1),y_label(2),y_label(3),strcat('Y_{',name_frame,'}'),'FontWeight','bold','color','k');
   text(z_label(1),z_label(2),z_label(3),strcat('Z_{',name_frame,'}'),'FontWeight','bold','color','k');
end;
end
