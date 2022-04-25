%% STATISTICS LAP
clear all
close all
% import data
% choose between:
% folder -> (assignment, robot-sim)
% arena -> (1-orignal_arnea, 2-fast_arena)
folder = 'robot-sim';
arena = '2-fast_arena';

textFileData = readtable(['../../', num2str(folder), '/stats/', num2str(arena), '/lap_time_', num2str(folder), '.txt']);
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
title(['Lap time [s] along the basic arena and mean value with standard deviation of: ', num2str(standardDeviation), ' [s]']);

figure;
hold on;
timeSorted = sort(lapTimes);
h = histogram(timeSorted, 'Normalization','pdf');

y = timeSorted(1):0.1:timeSorted(end);
mu = meanValue;
sigma = standardDeviation;
f = exp(-(y-mu).^2./(2*sigma^2))./(sigma*sqrt(2*pi));
plot(y,f,'LineWidth',2);

legend('Hisogram','Normal distribution');
title(['Comparison between histogram and normal distribution with mu = ', num2str(meanValue),' and sigma = ', num2str(standardDeviation)]);

% Lilliefors Test
lilTest = lillietest(lapTimes);