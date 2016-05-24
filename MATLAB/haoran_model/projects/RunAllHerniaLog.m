clc
close all
% 1
clear all
exp_name = 'egg_sponge_user04';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 2
clear all
exp_name = 'pistol_sponge_user03_stopped';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
% choose_save_video = 1;
% AnalysisHerniaSponge;
% movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 3
clear all
exp_name = 'pistol_sponge_user04';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
% choose_save_video = 1;
% AnalysisHerniaSponge;
% movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 4
clear all
exp_name = 'stylus_sponge_user03';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
% choose_save_video = 1;
% AnalysisHerniaSponge;
% movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 5
clear all
exp_name = 'user05_sponge_open';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 6
clear all
exp_name = 'user06_sponge_pistol';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 7
clear all
exp_name = 'user06_sponge_open';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 8
clear all
exp_name = 'user06_sponge_egg_periscope';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 9
clear all
exp_name = 'user05_sponge_stylus';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 10
clear all
exp_name = 'user05_sponge_pistol';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

% 11
clear all
exp_name = 'user05_sponge_periscope';
filename = strcat('C:\Users\haoranyu\Desktop\logs\',exp_name,'.txt');
data_name1 = strcat('..\export\',exp_name,'_tool_path_raw_with_angle.mat');
data_name2 = strcat('..\export\',exp_name,'_joint_log_hernia_setup_with_angle.mat');
data_name3 = strcat('..\export\',exp_name,'_tool_drive.mat');
ImportSpongeStylusToolPathV2;
load(data_name1);
choose_save_video = 1;
AnalysisHerniaSponge;
movie2avi(movie_frames,strcat(exp_name,'.avi'))
choose_save_video = 0;
AnalysisHerniaSponge;

