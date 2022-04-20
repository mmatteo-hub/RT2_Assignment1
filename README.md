# Statistics for the RT1 Assignment 1 branch <img src="https://user-images.githubusercontent.com/62358773/163468945-01c6131c-fc6b-434a-933d-4099714f7f17.png" width="5%" height="5%">

In this branch we performed a statistical analysis from the comparison of two different controllers for the robot of the [RT1 Assignment 1](https://github.com/mmatteo-hub/RT1_Assignment1).

### <img src="https://user-images.githubusercontent.com/62358773/163884096-27c17631-ff00-48f1-91bb-dfe8fac6384f.png" width="5%" height="5%"> Explanation of the work done
In our code we used a few functions to collect some data and analise them. We wanted to collect them by the use of a program written for that scope.
The statistical data we collected are:
* Lap time: the time of the lap completed by the robot calculated between two point identified as the start and the end for the statistics;
* Distance travelled by the robot: total distance travelled in terms of unit of the map;
* Number of corrections: total number of correction done by the robot in a single lap due to a wrong trajectory previously computed or due to some obstacles met along the path followed;

### <img src="https://user-images.githubusercontent.com/62358773/164229982-2ce36933-1949-4d6c-b0af-3ca14beeb1c5.png" width="5%" height="5%"> Possible errors in file writing
Since the threshold chosen to detect the robot lap time or the distance are intervals arbitrarly set there can be few errors while crossing theese areas thus bringing incorrections in files.
For example in some cases the time registered is near to a null-time and the same happens for the distance covered because the areas are controlled by boolean flags.
We report an example of errors below:

<img src="https://user-images.githubusercontent.com/62358773/164230676-0d23fec4-eb75-43ff-8f17-045de4d25bca.png" width="15%" height="15%">
<img src="https://user-images.githubusercontent.com/62358773/164230685-3c523ac6-f944-4284-a5cc-130a5bc560ac.png" width="25%" height="25%">

Theese data have to be deleted from the `.txt` file in order to avoid having fake data during the analisys.
