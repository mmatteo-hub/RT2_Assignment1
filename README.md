# Jupyter branch <img src="https://user-images.githubusercontent.com/62358773/157435593-53d7c0e1-919e-488c-845f-82988b838b20.png" width="5%" height="5%"></h2>

This branch is used to replace the previous UI (User Interface) provided by the [UI node](https://github.com/mmatteo-hub/RT2_Assignment1/blob/main/assignment/src/UI.cpp) of the code. The replacement is accomplished by the use of Jupyter, in particular with the use of a Notebook that acts like a UI and allows to have a better interface.
The functionalities that is will have are the same as the UI node, but in this case the graphical part is better: remember that the graphical part in the first version of the code was covered by the use of multiple shell windows each one running a specific node.

## <img src="https://user-images.githubusercontent.com/62358773/158230910-3ac2495f-208a-4e3c-a259-ab59f80e9d91.png" width="5%" height="5%"> Pre-requisites for Jupyter
### <img src="https://user-images.githubusercontent.com/62358773/158229723-84059fcb-d76e-41ad-a527-7b5e17a6fcd9.png" width="5%" height="5%"> Installation
In order to use the Jupyter tool inside our Docker image we have it installed (we were provided an image with it already installed by typing on the terminal `docker run -it --name my_jupyter -p 6080:80 -p 5900:5900 -p 8888:8888 carms84/noetic_ros2`. In this case we reserv a port (the 8888) and this will be useful for a future use.
The notebook will be visible inside the docker image by typing `http://localhost:8888` on the browser.
This will allow only the use of the *8888* port; if we want to use another post we will have to reserve it during the launch of the Docker image with the same specification as for the first one.
Be careful when creating a new image because we will have to re-install all the packages and extension useful.

### <img src="https://user-images.githubusercontent.com/62358773/158412884-3a11ce0f-560c-486b-ac46-af641413d6f0.png" width="5%" height="5%"> Visibility on the host side
In order to make the Notebook visible also out of the Docker image we can type `jupyter notebook --allow-root --ip 0.0.0.0` and we can type the same command for the visibility in our Browser, thus seen the same result.

### <img src="https://user-images.githubusercontent.com/62358773/158413804-bf7cde69-d192-4b9e-ac45-f1bc4165ab5f.png" width="5%" height="5%"> Jupyter Lab
JupyterLab is the next-generation user interface, including notebooks.
JupyterLab uses the same Notebook server and file format as the classic Jupyter Notebook to be fully compatible with the existing notebooks and kernels. The Classic Notebook and Jupyterlab can run side to side on the same computer. One can easily switch between the two interfaces.
We can install it by using `pip3 install jupyterlab` and make it visible on the host side by `jupyter lab --allow-root --ip 0.0.0.0`.

For JupyterLab we need to install Node.js by
`curl -sL https://deb.nodesource.com/setup_14.x -o nodesource_setup.sh` 
`bash nodesource_setup.sh`
`apt-get install -y nodejs`

#### <img src="https://user-images.githubusercontent.com/62358773/158414765-dfc2465d-e3a4-4813-a82e-0f27bf706416.png" width="2.5%" height="2.5%"> Extension
There are many extension that can be installed by using the *extension menù* that the Lab provides.

## <img src="https://user-images.githubusercontent.com/62358773/158417191-e4bd7959-d3cf-4e40-a724-8148367d9528.png" width="5%" height="5%"> Code
### <img src="https://user-images.githubusercontent.com/62358773/160856680-a9410ea2-6974-4750-bab5-e53238033494.png" width="5%" height="5%"> Launching the simulation
In order to launch the simulation we provided a `.launch` file to display only *Rviz* and *Gazebo* (even if this last one is not so useful).
For the robot simulation we have to type `roslaunc assignment assignment_jupyter.launch`.
Moreover, to use the *Jupyter* User Interface implemented we cam use the command written above (`jupyter notebook --allow-root --ip 0.0.0.0`).

### <img src="https://user-images.githubusercontent.com/62358773/160858871-3f3243f6-a0d6-42bb-9735-9f831b6f0d53.png" width="5%" height="5%"> Program explanation
Once bot the simulation and the Notebook are running we can interact with the robot through the UI provided.
We basically reproduced the same already present, so the main functionalities are four:
* Publish a position *(x,y)* that the robot has to reach;
  [figure of the pub pos]
* Drive the robot with the keyboard (here substituted by a graphics representing a keyboard);
  [figure of the keyboard]
* Cancel the current goal;
  [[figure of the cancel goal]]
* Quit the program.
  [figure of the quit]

The program is also provided a map representing the same thing of *Rviz*, to make the user able to know what the robot is doing also without the utilization of *Rviz*.

### <img src="https://user-images.githubusercontent.com/62358773/160860930-875e5445-89dd-4bec-9484-eeae7984dc67.png" width="5%" height="5%"> Modality enable
All the UI program is based on buttons that can be pressed multiple times, in particular for one modality, the manual use one, we have two steps to follow to use it:
* we first have to press the *enable button* that changes the value of a global variable `mod` that allows that selected modality to publish;
* later we can use that modality with the buttons below the first.

The other modalities has no enable button because they can publish periodically: of course, when the manual use is being used if another modality is selected by for example the publication of a goal, the `mod`variable is changed and the manual modality does not publish any longer thus avoiding overlapping of the infos published.

### <img src="https://user-images.githubusercontent.com/62358773/162995823-d51d500d-84c8-408c-b94d-5250846d1ee6.png" width=5% height=5%> Warning: correctness in execution
In order to avoid overloading the system and running into some problems of visualization, we highly recommend to start the Jupyter Notebook as follows:
* running it by terminal as explained in section above;
* once it opens on the web page start the *kernel* by pressing the dedicated botton:
 <img src="https://user-images.githubusercontent.com/62358773/162997563-028b91d1-b423-42eb-abec-3aa88319e7de.png" width=50% height=50%>

* run each cell separately, being carefull to wait the image being displayed. This because, since the Notebook runs on the Docker image and it is also related to the simlation on Rviz and Gazebo, there coulde be some errors in parsing the data.
* Once every cell has been started we can interact with them by the dedicated buttons properly displayed.
