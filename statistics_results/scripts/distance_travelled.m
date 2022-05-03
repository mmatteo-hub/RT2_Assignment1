%% STATISTICS DISTANCES
clear all
close all
% import data
% choose between:
% folder -> (assignment, robot-sim)
% arena -> (1-orignal_arnea, 2-fast_arena)
folder = 'assignment';
arena = '2-fast_arena';

textFileData = readtable(['../../', num2str(folder), '/stats/', num2str(arena), '/distance_travelled_', num2str(folder), '.txt']);
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
xlabel('Number of laps');
ylabel('Lap distance [units]');
legend('Distance Travelled', 'Mean Value');
title(['Lap distance [units] along the basic arena and mean value with standard deviation of: ', num2str(standardDeviation), ' [units]']);

figure;
hold on;
distancesSorted = sort(distances);
h = histogram(distancesSorted, 'Normalization','pdf');

y = distancesSorted(1):0.1:distancesSorted(end);
mu = meanValue;
sigma = standardDeviation;
f = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(y,f,'LineWidth',2);

legend('Hisogram','Normal distribution');
title(['Comparison between histogram and normal distribution with mu = ', num2str(meanValue),' and sigma = ', num2str(standardDeviation)]);

% Lilliefors Test
lilTest = lillietest(distances);