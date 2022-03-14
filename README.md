# Sphinx branch <img src="https://user-images.githubusercontent.com/62358773/157435708-d0d6175d-b03d-4c89-a63b-fe3ad1ff269f.png" width="5%" height="5%"></h2>
In this branch there is the documentation provided with the use of Sphinx. It is a particular tool that is installed inside the Docker image that allows the user to have a documentation in a `.html` file, more readable and accessible.

## <img src="https://user-images.githubusercontent.com/62358773/158230910-3ac2495f-208a-4e3c-a259-ab59f80e9d91.png" width="5%" height="5%"> Pre-requisites for Sphinx
### <img src="https://user-images.githubusercontent.com/62358773/158229723-84059fcb-d76e-41ad-a527-7b5e17a6fcd9.png" width="5%" height="5%"> Installation
In order to use Sphinx inside the Docker image we have to install it by typing: `apt-get install python3-sphinx`; moreover we have to install some extensions `pip3 install breathe` and `pip3 install sphinx-rtd-theme`.

### <img src="https://user-images.githubusercontent.com/62358773/158229276-daab681f-3322-4537-a913-043d805ede11.png" width="5%" height="5%"> Code preparation
The preparation of the code is the same as the Doxygen one so, to avoid the repetition, we suggest the visualization of the section [*Code Preparation*](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen) of the apposite Doxygen branch of the directory.

## <img src="https://user-images.githubusercontent.com/62358773/158230063-f844e068-8486-4eca-a694-20ee48a7234f.png" width="5%" height="5%"> Documentation generation
To start the creation of the documentation we have to type `sphinx-quickstart` on the terminal.
We are asked to insert:
* *Division of the folder [y/n]*: *n* (a common use is no type no to this question)
* *Name of the project*: the project name that will be present in the files that will be created
* *Author*: insert the author name
* *Project release*: insert the version of the code
* *Language [en]*: *en* (a common use is to set *en* = *English*)

The output we will see is the following:

<img src="https://user-images.githubusercontent.com/62358773/158257453-1bc4d4a8-519e-47e4-b94a-ec1f53899287.jpg" width="50%" height="50%">

After that we are ready to proceede with the creation of the documentation.
We have to open again the UI for Doxygen by the `doxywizard` shell command and we follow the same steps as for the Doxygen documentation ([here](https://github.com/mmatteo-hub/RT2_Assignment1/tree/doxygen) in the *Documentation generation* section).
We have to keep attention to some differences:
* the destination folder is *_build*;
* we untick the *HTML* and *Latex* options inside the menù but we tick the *XML* one;
* at the end of the preparation we do not have to run the Doxygen but to save the file as *Doxyfile.in* inside the package.

Once this step is over we have to modify another file that was created with the *sphinx-quickstart* command: the *conf.py* file.
We have to add:
```python
import os
import subprocess
import sys
sys.path.insert(0, os.path.abspath('.'))

subprocess.call('doxygen Doxyfile.in', shell=True)
```
just after the comments present.
Later we can add some useful extension in the apposit empty "array" present:
```python
extensions = [
	'sphinx.ext.autodoc',
	'sphinx.ext.doctest', 
	'sphinx.ext.intersphinx', 
	'sphinx.ext.todo', 
	'sphinx.ext.coverage', 
	'sphinx.ext.mathjax', 
	'sphinx.ext.ifconfig', 
	'sphinx.ext.viewcode', 
	'sphinx.ext.githubpages', 
	"sphinx.ext.napoleon", 
	'sphinx.ext.inheritance_diagram', 
	'breathe'
]
```
Very important is the section *Option for HTML output* where we have to inser:
```python
highlight_language = 'c++' 
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
```
in order to make the `C++` code readable by the *Sphinx* tool and to define the *theme* of the documentation.
At the end of the file we add
```python
breathe_projects = {
"RT2_Assignment1": "_build/xml/"
}
breathe_default_project = "RT2_Assignment1" 
breathe_default_members = ('members', 'undoc-members')
```
to set the project output ([*_build*](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx/assignment/_build) folder).

## <img src="https://user-images.githubusercontent.com/62358773/158230379-3c1a609e-e437-4c90-9f94-90f70d162e8a.png" width="5%" height="5%"> Directory organization
Inside this project the *destination directory* there is the [`_build`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx/assignment/_build) folder where we can find the three other directories [`html`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx/assignment/_build/html) and [`xml`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx/assignment/_build/xml) and [`doctrees`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx/assignment/_build/doctrees).

## <img src="https://user-images.githubusercontent.com/62358773/158228247-fe97068d-5dbd-431b-bc88-2f3b16f7dd63.png" width="5%" height="5%"> HTML result for the documentation

By double-clicking on *index.html* inside the [`html`](https://github.com/mmatteo-hub/RT2_Assignment1/tree/sphinx/assignment/_build/html) directory (or by opening it with a browser) we can see the result of the documentation provided by the Doxygen tool.
