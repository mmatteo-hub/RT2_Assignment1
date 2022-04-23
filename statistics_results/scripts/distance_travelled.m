%% STATISTICS DISTANCES
clear all
close all
% import data
textFileData = readtable('../../assignment/stats/1-original_arena/distance_travelled_assignment.txt');
arrayData = textFileData(:,5);
distances = table2array(arrayData);

% mean value
meanValue = sum(distances(:))/(length(distances));

% standard deviation
standardDeviation = std(distances);

figure;
hold on;
stairs(distances, 'b', 'LineWidth', 2)
plot([1 length(distances)], [meanValue meanValue], 'r-', 'LineWidth', 2);
xlabel('Number of laps')
ylabel('Lap time [s]')
legend('Lap Time', 'Mean Value')
title(['Lap distance [units] along the basic arena and mean value with standard deviation of: ', num2str(standardDeviation), ' [units]'])