%% STATISTICS DISTANCES
clear all
close all
% import data
textFileData = readtable('../../robot-sim/stats/2-fast_arena/distance_travelled_robot-sim.txt');
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
ylabel('Lap distance [units]')
legend('Distance Travelled', 'Mean Value')
title(['Lap distance [units] along the basic arena and mean value with standard deviation of: ', num2str(standardDeviation), ' [units]'])