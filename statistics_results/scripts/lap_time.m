%% STATISTICS LAP
clear all
close all
% import data
textFileData = readtable('../../assignment/stats/2-fast_arena/lap_time_assignment.txt');
arrayData = textFileData(:,2);
times = table2array(arrayData);
lapTimes = zeros(length(times)/3,1);

% select only lap time
for i=3:length(times)
    if(mod(i,3)== 0)
       lapTimes(i/3) = times(i);
    end
end

% mean value
meanValue = sum(lapTimes(:))/(length(lapTimes));

% standard deviation
standardDeviation = std(lapTimes);

figure;
hold on;
stairs(lapTimes, 'b', 'LineWidth', 2)
plot([1 length(lapTimes)], [meanValue meanValue], 'r-', 'LineWidth', 2);
xlabel('Number of laps')
ylabel('Lap time [s]')
legend('Lap Time', 'Mean Value')
title(['Lap time [s] along the basic arena and mean value with standard deviation of: ', num2str(standardDeviation), ' [s]'])