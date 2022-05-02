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

textFileData1 = readtable(['../../', num2str(folder1), '/stats/', num2str(arena1), '/general_lap_time_', num2str(folder1), '.txt']);
arrayData1 = textFileData1(:,3);
times1 = table2array(arrayData1);
lapTimes1 = zeros(length(times1)/3,1);

textFileData2 = readtable(['../../', num2str(folder2), '/stats/', num2str(arena2), '/general_lap_time_', num2str(folder2), '.txt']);
arrayData2 = textFileData2(:,3);
times2 = table2array(arrayData2);
lapTimes2 = zeros(length(times2)/3,1);

% select only lap time
for i=3:length(times1)
    if(mod(i,3)== 0)
       lapTimes1(i/3) = times1(i);
    end
end

% select only lap time
for i=3:length(times2)
    if(mod(i,3)== 0)
       lapTimes2(i/3) = times2(i);
    end
end

% Lilliefors Test
lilTest1 = lillietest(lapTimes1);
lilTest2 = lillietest(lapTimes2);

if(lilTest1 == 1 || lilTest2 == 1) % not from normal distribution
    [p,h] = ranksum(lapTimes1,lapTimes2);

elseif(lilTest1 == 0 && lilTest2 == 0) % both from normal distribution
    h_tt = ttest(lapTimes1,lapTimes2);
end

C = [lapTimes1', lapTimes2'];
grp = [zeros(1,length(lapTimes1)),ones(1,length(lapTimes2))];
boxplot(C,grp,'Labels',{'assignment','robot-sim'},'Whisker',1);
xlabel('Groups')
ylabel('Lap Time [s]')
title('Boxplot between lap times');

