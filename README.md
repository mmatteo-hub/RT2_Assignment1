# Doxygen branch <img src="https://user-images.githubusercontent.com/62358773/157435494-aad1604c-ecde-4b38-aa5e-13cef84f4620.png" width="5%" height="5%"></h2>

In this branch there is the documentation provided with the use of Doxygen. It is a particular tool that is installed inside the Docker image that allows the user to have a documentation in a `.html` file, more readable and accessible.

## Pre-requisites for Doxygen
### <img src="https://user-images.githubusercontent.com/62358773/158229723-84059fcb-d76e-41ad-a527-7b5e17a6fcd9.png" width="5%" height="5%"> Installation
In order to use Doxygen inside the Docker image we have to install it by typing: `apt-get instal -y doxygen`; moreover it is provided of a GUI (Graphical User Interface) to make the documentation building simpler: this can be installed by typing `atp-get install doxygen-gui`.

### <img src="https://user-images.githubusercontent.com/62358773/158229276-daab681f-3322-4537-a913-043d805ede11.png" width="5%" height="5%"> Code preparation
Doxygen is used for the `C++` files, but they have to be "prepared" in such a way the program can choose and take the right part to document.
Without entering in the details of the documentation building, we have to add particular comments in a proper format to the code:
For each function (we want to document) we have to add:
```cpp
/**
 * \brief Brief description of the function
 * \param Parameters taken as input by the function
 * 
 * \return Return value given by the function
 * 
 * "Description of the function: how it works etc ..."
 * 
**/
```
Moreover, we can add a general comment for the entire file:
```cpp
/**
 * \file Name of the .cpp file
 * \brief Brief description of the .cpp file
 * \author Name of the authore
 * \version	Code version
 * \date Date
 * 
 * \details
 * 
 * Subscribers to: <BR>
 * All the subscribers used by the program 
 *
 * Publisher to: <BR>
 * All the publishers used by the program 
 * 
 * Services: <BR>
 * All the services used by the program
 * 
 * Description:
 * 
 * Description of the program: how it works etc ...
 * 
**/
```
Once we have done this for all our files and for all the function we want to document we are ready.

### <img src="https://user-images.githubusercontent.com/62358773/158230063-f844e068-8486-4eca-a694-20ee48a7234f.png" width="5%" height="5%"> Documentation generation
To generate the documentation we have to call the tool `doxywizard` by the terminal that allows us to have an interface to complete the documentation building.
We have that kind of window:

<img src="https://user-images.githubusercontent.com/62358773/158227235-e41e1232-3aee-4503-8097-d630dba64f98.jpg" width=50% height=50%>

Once defined all the fieds with the proper arguments can press the `next` button and `run` (at the end of the process).
We can see that in the folder we specified as *destination folder* we will have two different folders:
* html (html documentation)
* latex (latex documentation)

### <img src="https://user-images.githubusercontent.com/62358773/158230379-3c1a609e-e437-4c90-9f94-90f70d162e8a.png" width="5%" height="5%"> Directory organization
Inside this project the *destination directory* is the [`docs`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen/assignment/docs) folder where we can find the two other directories [`html`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen/assignment/docs/html) and [`latex`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen/assignment/docs/latex).

### <img src="https://user-images.githubusercontent.com/62358773/158228247-fe97068d-5dbd-431b-bc88-2f3b16f7dd63.png" width="5%" height="5%"> HTML result for the documentation

By double-clicking on *index.html* inside the [`html`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen/assignment/docs/html) directory (or by opening it with a browser) we can see the result of the documentation provided by the Doxygen tool.
