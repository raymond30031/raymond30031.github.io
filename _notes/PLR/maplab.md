### Clone repo
setup the public key following this instruction:
https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
https://help.github.com/articles/error-permission-denied-publickey/

### environment setup with conda
I decided to use the test as i believe it is the same one for maskrcnn

To load the python modules, run: 
'source activate test'

Note that ROS uses python 2.7 and the conda env test has python 3.6.5
so it makes checking things a bit troublesome as both path is in the $PYTHONPATH

So keep the ros related missing package to the 2.7 by doing pip install with the conda env deactivated

### Fixing build errors

#### rovio package
No module named 'em' error:
this is a dependency package of ROS or catkin, but it is not automatically installed when installing the ROS packages

soltuion:
run 'pip install empy'

reference:
https://github.com/ros/genmsg/issues/63

#### ceres_catkin

The following problems is caused by using Python3

#### plotty package
wchar_t error:
replace 'char' with 'wchar_t' and add 'L' right before the string name to make it a wide string

PyString error:
replace 'PyString_FromString' to 'PyBytes_FromString'

#### maplab_common
error: ‘PyString_FromString’ was not declared in this scope

solution:
replace 'PyString_FromString' to 'PyBytes_FromString'

### using virtual python environment

####create a virtual environment
virtualenv --python=python3.5 venv3.5

#### activate and deactivate virtualenv
to activate: source venv3.5/bin/activate
to deactivate: deactivate



