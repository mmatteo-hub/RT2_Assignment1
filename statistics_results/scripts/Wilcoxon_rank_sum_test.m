clear all
close all
clear all
close all
% import data
% choose between:
% folder -> (assignment, robot-sim)
% arena -> (1-orignal_arnea, 2-fast_arena)
folder1 = 'assignment';
arena1 = '1-original_arena';
folder2 = 'robot-sim';
arena2 = '1-original_arena';

textFileData1 = readtable(['../../', num2str(folder1), '/stats/', num2str(arena1), '/distance_travelled_', num2str(folder1), '.txt']);
arrayData1 = textFileData1(:,5);
distances1 = table2array(arrayData1);

textFileData2 = readtable(['../../', num2str(folder2), '/stats/', num2str(arena2), '/distance_travelled_', num2str(folder2), '.txt']);
arrayData2 = textFileData2(:,5);
distances2 = table2array(arrayData2);

% test
[p,h] = ranksum(distances1,distances2);