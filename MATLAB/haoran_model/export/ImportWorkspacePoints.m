clc
clear all
close all
% Load hernia workspace model
vertex_hernia_macro_workspace = stl2matlab('workspace\Ventral_Hernia_Workspace_Refined.STL');
scale_mm_to_m = 1/1000;
vertex_hernia_macro_workspace{1} = vertex_hernia_macro_workspace{1} * scale_mm_to_m;
vertex_hernia_macro_workspace{2} = vertex_hernia_macro_workspace{2} * scale_mm_to_m;
vertex_hernia_macro_workspace{3} = vertex_hernia_macro_workspace{3} * scale_mm_to_m;
figure (1)
view(3)
PlotStl(vertex_hernia_macro_workspace,[0.5 0.5 0.5 1])
hold on
for index = 1 : length(vertex_hernia_macro_workspace{1})
    point_clouds_hernia_workspace(:,index * 3 - 2) = [vertex_hernia_macro_workspace{1}(1,index);vertex_hernia_macro_workspace{2}(1,index);vertex_hernia_macro_workspace{3}(1,index)];
    point_clouds_hernia_workspace(:,index * 3 - 1) = [vertex_hernia_macro_workspace{1}(2,index);vertex_hernia_macro_workspace{2}(2,index);vertex_hernia_macro_workspace{3}(2,index)];
    point_clouds_hernia_workspace(:,index * 3 - 0) = [vertex_hernia_macro_workspace{1}(3,index);vertex_hernia_macro_workspace{2}(3,index);vertex_hernia_macro_workspace{3}(3,index)];
    plot3(point_clouds_hernia_workspace(1,index * 3 - 2),point_clouds_hernia_workspace(2,index * 3 - 2),point_clouds_hernia_workspace(3,index * 3 - 2),'MarkerSize',10,'Marker','*');
    plot3(point_clouds_hernia_workspace(1,index * 3 - 1),point_clouds_hernia_workspace(2,index * 3 - 1),point_clouds_hernia_workspace(3,index * 3 - 1),'MarkerSize',10,'Marker','*');
    plot3(point_clouds_hernia_workspace(1,index * 3 - 0),point_clouds_hernia_workspace(2,index * 3 - 0),point_clouds_hernia_workspace(3,index * 3 - 0),'MarkerSize',10,'Marker','*');
    
end
drawnow;
save('export\point_clouds_hernia_workspace.mat','point_clouds_hernia_workspace');