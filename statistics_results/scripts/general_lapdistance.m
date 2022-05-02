%% GENERAL ALGORITHM (LAP TIME)

clear all
close all
% import data
% choose between:
% folder -> (assignment, robot-sim)
% arena -> (1-orignal_arnea, 2-fast_arena)
folder1 = 'assignment';
arena1 = '0-general';
folder2 = 'robot-sim';
arena2 = '0-general';

textFileData1 = readtable(['../../', num2str(folder1), '/stats/', num2str(arena1), '/general_distance_travelled_', num2str(folder1), '.txt']);
arrayData1 = textFileData1(:,5);
distances1 = table2array(arrayData1);

textFileData2 = readtable(['../../', num2str(folder2), '/stats/', num2str(arena2), '/general_distance_travelled_', num2str(folder2), '.txt']);
arrayData2 = textFileData2(:,5);
distances2 = table2array(arrayData2);

% Lilliefors Test
lilTest1 = lillietest(distances1);
lilTest2 = lillietest(distances2);

if(lilTest1 == 1 || lilTest2 == 1) % not from normal distribution
    [p,h] = ranksum(distances1,distances2);

elseif(lilTest1 == 0 && lilTest2 == 0) % both from normal distribution
    h_tt = ttest(distances1,distances2);
end

C = [distances1', distances2'];
grp = [zeros(1,length(distances1)),ones(1,length(distances2))];
boxplot(C,grp,'Labels',{'assignment','robot-sim'},'Whisker',1);
xlabel('Groups')
ylabel('Lap Distance [m]')
title('Boxplot between lap distance');

