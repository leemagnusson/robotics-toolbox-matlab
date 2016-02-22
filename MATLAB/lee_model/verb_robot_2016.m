% verb_robot2016.m
% Lee Magnusson
% 2/1/16

close all;

global r 

robot_dir = 'robot 2016'

baseT = r2t(rotz(-pi/2)*rotx(pi/2));
[r,other_param] = create_robot(robot_dir,baseT)

initial_robot_tests(r,other_param,robot_dir);
