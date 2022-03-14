# Research Track2: Assignment #1
## <img src="https://user-images.githubusercontent.com/62358773/158240285-e5cf386b-233d-445e-ab4a-1a14976d39a4.png" width="5%" height="5%"> Code documentation and new <img src="https://user-images.githubusercontent.com/62358773/158241494-e5a35341-0032-4ecf-b57d-bb0d8a4c1814.png" width="5%" height="5%"> UI (User Interface) for the RT1 Assignment 3
### <img src="https://user-images.githubusercontent.com/62358773/158238820-f418cc09-4227-4afc-9c31-1705dfb64f5a.png" width="5%" height="5%"> Professor [Carmine Recchiuto](https://github.com/CarmineD8), <img src="https://user-images.githubusercontent.com/62358773/158238810-c5dcb486-ba24-4b35-87de-39a54e88f36b.png" width="5%" height="5%"> Student: [Matteo Maragliano](https://github.com/mmatteo-hub)

## Master Branch <img src="https://user-images.githubusercontent.com/62358773/157435327-929ef025-418e-43d3-9056-1f21f7e7b1e9.png" width="5%" height="5%"></h2>

### <img src="https://user-images.githubusercontent.com/62358773/158230379-3c1a609e-e437-4c90-9f94-90f70d162e8a.png" width="5%" height="5%"> Repository organization
The repository is divided into 4 different branches (the site for the complete documentation of each tool is linked):
* <img src="https://user-images.githubusercontent.com/62358773/158239721-23bf05e3-96de-4e5c-8b72-f95d75400b33.png" width="2.5%" height="2.5%"> **main**
* <img src="https://user-images.githubusercontent.com/62358773/157435494-aad1604c-ecde-4b38-aa5e-13cef84f4620.png" width="2.5%" height="2.5%"> [**Doxygen**](https://www.doxygen.nl/manual/docblocks.html)
* <img src="https://user-images.githubusercontent.com/62358773/157435593-53d7c0e1-919e-488c-845f-82988b838b20.png" width="2.5%" height="2.5%"> [**Jupyter**](https://jupyter.org)
* <img src="https://user-images.githubusercontent.com/62358773/157435708-d0d6175d-b03d-4c89-a63b-fe3ad1ff269f.png" width="2.5%" height="2.5%"> [**Sphinx**](https://www.sphinx-doc.org/en/master/)

Each one represents a different part for the documentation obtained in a different way.
In particular:
* [**main**](https://github.com/mmatteo-hub/RT2_Assignment1) : it is the main branch with the original project code taken from the [RT1 Assignment3](https://github.com/mmatteo-hub/RT1_Assignment3/tree/noetic);
* [**doxygen**](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen) : it is the branch with the *doxygen* documentation. All node functions are provided particular comment structure to allow the program working properly;
* [**jupyter**](https://github.com/mmatteo-hub/RT2_Assignment1/tree/jupyter) : it is the branch in which the UI is replaced by the *Jupyter* use;
* [**sphinx**](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx) : it is the branch with the *sphinx* documentation. It is provided the same code as *doxygen*-branch in order to make the *sphinx* program working properly.

Each of the three different branches are explained in their branch respectively.

### <img src="https://user-images.githubusercontent.com/62358773/157433800-cbf3f310-46d9-4bdf-9afb-6993a2045f9f.png" width="5%" height="5%"> Switching the branch 
In order to switch the branch and to see the documentation for each one we have to use a temrminal window or a GUI providing the same functionaloties.
We have to be in out directory so : `cd [path]/RT2_Assignment1`; then we have to check the branch we are in by: `git branch`.
To switch from a branch to another we will type: `git chekout [branch name]` where the `[branch name]` parameter has to be chosen by the 4 different branches the reposotory has.
The change of branch is a key step to access the code in the Doker image and see the result of the compilation.
