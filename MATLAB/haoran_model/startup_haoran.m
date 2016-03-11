addpath([pwd, '/../3rdparty/robotics-toolbox-matlab']);
addpath([pwd,'/../3rdparty/toolbox-common-matlab']);
addpath([pwd,'/../3rdparty/jsonlab-1.2/jsonlab']);
% parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 15 8 11;0 18 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
parent_number=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 14 7 10;0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17];
save('parent_number.mat','parent_number');