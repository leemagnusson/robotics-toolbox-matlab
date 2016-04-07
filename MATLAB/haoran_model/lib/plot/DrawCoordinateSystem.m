% function DrawCoordinateSystem(axis_length,rotation,translation,color,name_frame)
% this function draws a coordinate system in the requested orientation given by rotation matrix R 
% and at a specific origin
% example: draw_coordinate_system(10,rotd([0,0,1],45),[10,10,10]','rgb')
% uses functions rotd() and cone() and matlab function cylinder()
% copyright Nabil Simaan, 2003
% modified Haoran Yu, 2016
%---------------------------------------------------------------------------
function DrawCoordinateSystem(axis_length,rotation,translation,color,name_frame)
hold on;

x_axis_unit_vec = rotation(:,1);  %the axis directions in WCS
y_axis_unit_vec = rotation(:,2);

x_axis = axis_length(1)*rotation(:,1); %end point of x_axis in wcs
y_axis = axis_length(2)*rotation(:,2);
z_axis = axis_length(3)*rotation(:,3);

%drawing the axis lines
line([translation(1);translation(1)+x_axis(1)],[translation(2);translation(2)+x_axis(2)],[translation(3);translation(3)+x_axis(3)],'color',color(1),'linewidth',2);
line([translation(1);translation(1)+y_axis(1)],[translation(2);translation(2)+y_axis(2)],[translation(3);translation(3)+y_axis(3)],'color',color(2),'linewidth',2);
line([translation(1);translation(1)+z_axis(1)],[translation(2);translation(2)+z_axis(2)],[translation(3);translation(3)+z_axis(3)],'color',color(3),'linewidth',2);

% setting cone variables
cone_height = axis_length(1)/5;
cone_base_radius = axis_length(1)/10;
numpoints = 20;

%drawing the cone for x_axis%
Rx=rotd(y_axis_unit_vec,90)*rotation; %since I am specifying all my rotations about axes specified in WCS I premultiply
                               %same result can be obtained by Rx=R*rotd([0;1;0],90), where I am rotating about the y axis given in the new system
x_cone_origin=[translation(1)+x_axis(1); translation(2)+x_axis(2); translation(3)+x_axis(3)];
[X,Y,Z]=cone(cone_height,cone_base_radius,numpoints,Rx,x_cone_origin);
surf(X,Y,Z,'EdgeColor',color(1),'FaceColor',color(1),'EdgeLighting','phong','FaceLighting','phong');

%drawing the cone for y_axis%
Ry=rotd(x_axis_unit_vec,-90)*rotation;
y_cone_origin=[translation(1)+y_axis(1); translation(2)+y_axis(2); translation(3)+y_axis(3)];
[X,Y,Z]=cone(cone_height,cone_base_radius,numpoints,Ry,y_cone_origin);
surf(X,Y,Z,'EdgeColor',color(2),'FaceColor',color(2),'EdgeLighting','phong','FaceLighting','phong');


%drawing the cone for z_axis%
z_cone_origin=[translation(1)+z_axis(1); translation(2)+z_axis(2); translation(3)+z_axis(3)];
[X,Y,Z]=cone(cone_height,cone_base_radius,numpoints,rotation,z_cone_origin);
surf(X,Y,Z,'EdgeColor',color(3),'FaceColor',color(3),'EdgeLighting','phong','FaceLighting','phong');

if nargin==5, %add text labels
   x_label_pos= x_cone_origin + 1.8*cone_height*x_axis/axis_length(1); %using unit vector 
   y_label_pos= y_cone_origin + 1.8*cone_height*y_axis/axis_length(2);
   z_label_pos= z_cone_origin + 1.8*cone_height*z_axis/axis_length(3);
   %disp(';ls;ls;ls;');
   text(x_label_pos(1),x_label_pos(2),x_label_pos(3),strcat('X_{',name_frame,'}'),'FontWeight','bold','color','k');
   text(y_label_pos(1),y_label_pos(2),y_label_pos(3),strcat('Y_{',name_frame,'}'),'FontWeight','bold','color','k');
   text(z_label_pos(1),z_label_pos(2),z_label_pos(3),strcat('Z_{',name_frame,'}'),'FontWeight','bold','color','k');
end;
return;


%--------------------------------------------------------------------------------------------
% function cone(cone_height,cone_base,numpoints,R,cone_origin) returns the [X,Y,Z] 
% such that surf([X,Y,Z]) draws a cone in the specific orientation given by rotation matrix R
% and in origin specified by cone_origin
% written by nabil simaan Nov. 25, 2003.
%---------------------------------------------------------------------------------------------

function [X,Y,Z]=cone(cone_height,cone_base_radius,numpoints,R,cone_origin)

[X,Y,Z]=cylinder([cone_base_radius,0],numpoints);  %calculating a cone 1 unit in height and centered at [0,0,0] with axis in Z direction%
Z=Z.*cone_height;

Cone_base=[];
Cone_top=[];
for i=1:1:numpoints+1,
    Cone_base(:,i)=[X(1,i);Y(1,i);Z(1,i)];
    Cone_top(:,i)=[X(2,i);Y(2,i);Z(2,i)];
    
    Cone_base(:,i)=R*Cone_base(:,i)+cone_origin;  %rotating the cone base and translating
    Cone_top(:,i)=R*Cone_top(:,i)+cone_origin;  %rotating the cone top points and translating
    
    X(1,i)=Cone_base(1,i);
    X(2,i)=Cone_top(1,i);
    
    Y(1,i)=Cone_base(2,i);
    Y(2,i)=Cone_top(2,i);
    
    Z(1,i)=Cone_base(3,i);
    Z(2,i)=Cone_top(3,i);
end;
return;
%------- function r = rotd(n,phi); -----------------
% computes the rotation matrix for rotation about axis n phi degrees
%-----------------------------------------------------------
function r = rotd(n,phi)
  phi=phi*pi/180;  
  n = n ./ norm(n);
  s = phi*[0 -n(3) n(2);n(3) 0 -n(1);-n(2) n(1) 0];
  r = expm(s);
return;





