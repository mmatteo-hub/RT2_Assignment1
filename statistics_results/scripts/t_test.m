%% T-TEST

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

textFileData2 = readtable(['../../', num2str(folder1), '/stats/', num2str(arena1), '/lap_time_', num2str(folder1), '.txt']);
arrayData1 = textFileData2(:,2);
times1 = table2array(arrayData1);
lapTimes1 = zeros(length(times1)/3,1);

textFileData2 = readtable(['../../', num2str(folder2), '/stats/', num2str(arena2), '/lap_time_', num2str(folder2), '.txt']);
arrayData2 = textFileData2(:,2);
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

h_tt = ttest(lapTimes1,lapTimes2);

C = [lapTimes1', lapTimes2'];
grp = [zeros(1,length(lapTimes1)),ones(1,length(lapTimes2))];
boxplot(C,grp,'Labels',{'assignment','robot-sim'},'Whisker',1);
xlabel('Groups')
ylabel('Lap Time [s]')
title('Boxplot between lap times');
