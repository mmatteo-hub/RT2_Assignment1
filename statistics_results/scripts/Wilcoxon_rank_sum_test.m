%% WILCOXON RANK SUM TEST

clear all
close all
% import data
% choose between:
% folder -> (assignment, robot-sim)
% arena -> (1-orignal_arnea, 2-fast_arena)
folder1 = 'assignment';
arena1 = '2-fast_arena';
folder2 = 'robot-sim';
arena2 = '2-fast_arena';

textFileData1 = readtable(['../../', num2str(folder1), '/stats/', num2str(arena1), '/distance_travelled_', num2str(folder1), '.txt']);
arrayData1 = textFileData1(:,5);
distances1 = table2array(arrayData1);

textFileData2 = readtable(['../../', num2str(folder2), '/stats/', num2str(arena2), '/distance_travelled_', num2str(folder2), '.txt']);
arrayData2 = textFileData2(:,5);
distances2 = table2array(arrayData2);

% test
[p,h] = ranksum(distances1,distances2);

C = [distances1', distances2'];
grp = [zeros(1,length(distances1)),ones(1,length(distances2))];
boxplot(C,grp,'Labels',{'assignment','robot-sim'},'Whisker',1);
xlabel('Groups')
ylabel('Lap distances [units]')
title('Boxplot between lap distances');