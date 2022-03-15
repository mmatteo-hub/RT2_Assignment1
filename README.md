# Jupyter branch <img src="https://user-images.githubusercontent.com/62358773/157435593-53d7c0e1-919e-488c-845f-82988b838b20.png" width="5%" height="5%"></h2>

This branch is used to replace the previous UI (User Interface) provided by the [UI node](https://github.com/mmatteo-hub/RT2_Assignment1/blob/main/assignment/src/UI.cpp) of the code. The replacement is accomplished by the use of Jupyter, in particular with the use of a Notebook that acts like a UI and allows to have a better interface.
The functionalities that is will have are the same as the UI node, but in this case the graphical part is better: remember that the graphical part in the first version of the code was covered by the use of multiple shell windows each one running a specific node.

## <img src="https://user-images.githubusercontent.com/62358773/158230910-3ac2495f-208a-4e3c-a259-ab59f80e9d91.png" width="5%" height="5%"> Pre-requisites for Jupyter
### <img src="https://user-images.githubusercontent.com/62358773/158229723-84059fcb-d76e-41ad-a527-7b5e17a6fcd9.png" width="5%" height="5%"> Installation
In order to use the Jupyter tool inside our Docker image we have it installed (we were provided an image with it already installed by typing on the terminal `docker run -it --name my_jupyter -p 6080:80 -p 5900:5900 -p 8888:8888 carms84/noetic_ros2`. In this case we reserv a port (the 8888) and this will be useful for a future use.
The notebook will be visible inside the docker image by typing `http://localhost:8888` on the browser.
#### <img src="https://user-images.githubusercontent.com/62358773/158412884-3a11ce0f-560c-486b-ac46-af641413d6f0.png" width="5%" height="5%"> Visibility on the host side
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
