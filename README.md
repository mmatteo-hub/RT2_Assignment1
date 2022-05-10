# Statistics for the RT1 Assignment 1 branch <img src="https://user-images.githubusercontent.com/62358773/163468945-01c6131c-fc6b-434a-933d-4099714f7f17.png" width="5%" height="5%">

In this branch we performed a statistical analysis from the comparison of two different controllers for the robot of the [RT1 Assignment 1](https://github.com/mmatteo-hub/RT1_Assignment1).

## <img src="https://user-images.githubusercontent.com/62358773/163884096-27c17631-ff00-48f1-91bb-dfe8fac6384f.png" width="5%" height="5%"> Explanation of the work done
In our code we used a few functions to collect some data and analise them. We wanted to collect them by the use of a program written for that scope.
The statistical data we collected are:
* Lap time: the time of the lap completed by the robot calculated between two point identified as the start and the end for the statistics;
* Distance travelled by the robot: total distance travelled in terms of unit of the map;

These statistical analysis have been teste among two different arenas for an amount of 30 iterations per arena and per controller; then in order to test the best algorithm in general, of course among one of the data acquired, we modified the original arena by moving the obstacles and then acquiring the same data as for the first test.

All the results and comment have been reported into an apposite report.

## <img src="https://user-images.githubusercontent.com/62358773/158230379-3c1a609e-e437-4c90-9f94-90f70d162e8a.png" width="5%" height="5%"> Repository organization
The repository has two main folders:
* *assignment*: containing all the files useful to run the code with the assignment controller proposed in RT1;
* *robot-sim*: containing all the files useful to run the code with the robot-sim controller provided by the Professor to have a comparison.

Each one of these folder contains a *stats* folder containing all the statistical analisys computed and the relatex `.txt`files with the data recorded.
All the data are divided into subfolders named with a significant name to let the user know what he is analisying.

## <img src="https://user-images.githubusercontent.com/62358773/158417191-e4bd7959-d3cf-4e40-a724-8148367d9528.png" width="5%" height="5%"> Code
### <img src="https://user-images.githubusercontent.com/62358773/160856680-a9410ea2-6974-4750-bab5-e53238033494.png" width="5%" height="5%"> Launching the simulation
The main folder contains a shell script to run the code: `run.sh`, it a parameter which is the name of the folder we want to run the code of.
In order to start the simulation we have to type: `./run.sh _name_` where *_name_* is:
* assignment
* robot-sim

### <img src="https://user-images.githubusercontent.com/62358773/164914022-5eff9f0f-eedb-4919-b29d-c362391b5ac0.png" width="5%" height="5%"> Library for the statistics
In order to avoid including too many additions to the origin code it was provided a library in which there are all the functions needed to do statistics.
The most important thing is to change just one parameter: `stats_path_name` which is a *string* specifying the folder in which we have the file we want to write into; missing this step brings to errors in file writing since we will combine data that are not related to each other.

Thanks to this we can just add to the program few lines of code in order to have less modifications as possible.
 
### <img src="https://user-images.githubusercontent.com/62358773/164229982-2ce36933-1949-4d6c-b0af-3ca14beeb1c5.png" width="5%" height="5%"> Possible errors in file writing
Since the threshold chosen to detect the robot lap time or the distance are intervals arbitrarly set there can be few errors while crossing theese areas thus bringing incorrections in files.
For example in some cases the time registered is near to a null-time and the same happens for the distance covered because the areas are controlled by boolean flags.
We report an example of errors below:

<img src="https://user-images.githubusercontent.com/62358773/164230676-0d23fec4-eb75-43ff-8f17-045de4d25bca.png" width="15%" height="15%">
<img src="https://user-images.githubusercontent.com/62358773/164230685-3c523ac6-f944-4284-a5cc-130a5bc560ac.png" width="25%" height="25%">

Theese data have to be deleted from the `.txt` file in order to avoid having fake data during the analisys.

## <img src="https://user-images.githubusercontent.com/62358773/164914443-71e8db15-e1bf-42d3-babe-71077f0f9c72.png" width="5%" height="5%"> Results
The statistical results have been tested and obtained by MATLAB.
In order to try and see the code there is a folder named `scripts` in which there are the files used.

The only thing it is important to change when the code is run is to check:
* folder: chosen between *assignment* and *robot-sim*;
* arena: chosen between *1-original_arena* and *2-fast_arena*

The output is 2 images and a variable to be read from the Workspace.
In order to understand the results it is suggested to read the brief [report](https://github.com/mmatteo-hub/RT2_Assignment1/blob/statistics_RT1_Assignment1/statistics_results/report/RT2__Report_RT2_Assignment1_pt2.pdf) which explains all the procedure of data analysis.
