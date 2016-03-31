function new_vertices = apply_transform(matrix,vertices)

vertices = [vertices';ones(1,size(vertices,1))];

new_vertices = (matrix*vertices)';
new_vertices = new_vertices(:,1:3);