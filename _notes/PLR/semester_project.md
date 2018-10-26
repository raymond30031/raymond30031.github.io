###build
catkin clean
catkin build maplab
catkin build --no-deps semantify_plugin
catkin build pybind11_catkin # need this until we add it to auto build

### Setting up matterport maskrcnn with gpu and virtualenv
 git clone https://github.com/matterport/Mask_RCNN.git

### Run sample
jupyter notebook in command line then open sample.ipynb

####create a virtual environment
virtualenv --python=python3.5 venv3.5

#### install from requirement
cd to the directory where requirements.txt is located.
activate your virtualenv.
run: pip install -r requirements.txt in your shell.

#### activate and deactivate virtualenv
to activate: source venv3.5/bin/activate

### Setup Errors
####Error: no module pycocotool
Solution:

Step 1:
git clone https://github.com/waleedka/coco

Step 2: edit the makefile for conda env
'''
all:
# install pycocotools locally
/home/jkuo/miniconda3/envs/test/bin/python setup.py build_ext --inplace
rm -rf build

install:
# install pycocotools to the Python site-packages
/home/jkuo/miniconda3/envs/test/bin/python setup.py build_ext install
rm -rf build
'''

Step 3: Run make install in the PythonAPI folder and make sure the python is from conda env

'make install'
'python setup.py install'

Step 4:install notebook for the conda env specifically, so the kernel knows to look into the paths of this env/lib, instead of the miniconda/lib
conda install notebook

Ref:
https://github.com/matterport/Mask_RCNN/issues/6

####fatal error: Python.h: No such file or directory
Solution:

for virtualenv it copies the system files into a fold and link python to them.
So if python dev is not installed before it won't have it in the folder

sudo apt-get update
sudo apt install -y python3-pip
sudo apt install build-essential libssl-dev libffi-dev python3-dev

Ref:
https://www.digitalocean.com/community/tutorials/how-to-install-python-3-and-set-up-a-programming-environment-on-an-ubuntu-16-04-server

####ERROR
E tensorflow/stream_executor/cuda/cuda_dnn.cc:343] Loaded runtime CuDNN library: 7.0.5 but source was compiled with: 7.2.1.  CuDNN library major and minor version needs to match or have higher minor version in case of CuDNN 7.0 or later version. If using a binary install, upgrade your CuDNN library.  If building from sources, make sure the library loaded at runtime is compatible with the version specified during compile configuration.

solution: downgrade tensorflow to 1.8

pip install tensorflow-gpu==1.8

Reference:
https://stackoverflow.com/questions/49960132/cudnn-library-compatibility-error-after-loading-model-weights
https://stackoverflow.com/questions/50622525/which-tensorflow-and-cuda-version-combinations-are-compatible

### build plugin
catkin build --no-deps semantify_plugin

need to source everytime you build something

### run maplab
rosrun maplab_console maplab_console -v=1

### load map
load --map_folder /media/jkuo/Data/semester_project/EuRoC/maps/MH_01_easy

### plugins

#### list plugins
type help in console

ref:
https://github.com/ethz-asl/maplab_private/wiki/Console-Plugin-System

## Problems:
### what is the difference b/w PYTHONPATH and PYTHONHOME env variable

ref:
https://docs.python.org/3.5/using/cmdline.html

### std::string to char*
https://stackoverflow.com/questions/7352099/stdstring-to-char

### ImportError: No module named site
so this is a problem when the python.h and the pythonpath has different version
tried to set it with Py_SetPythonHome Py_SetPythonPath Py_SetProgramName at run time, 
but it does not work. the run time PYTHONPATH remains pointed to the default one.
Why? (missing reference, andrei found it, so i don't have it)
Because apparently the env var present in the terminal (from source/devel.setup) cannot be overwritten 
by the ones we set at runtime. then what is the points of these functions?
dead end for us because ros requires py2.7, but matterport/maskrcnn req py3

Turns out the matterport/maskrcnn req 3 mainly becuz of pycocotools and some util functions
When inference you can remove these dependency and run.
Only problem is the division changed between py2.7 and 3.5 and it affects inference
see the error below for details


https://stackoverflow.com/questions/10675315/setting-pythonpath-and-pythonhome
https://stackoverflow.com/questions/27104140/embedding-python-no-module-named-site

### setting up env for py2.7
This prevents tensorflow from being installed
error: Cannot uninstall 'enum34'. It is a distutils installed project

solution: ignores the installed enum34
sudo pip install --ignore-installed tensorflow

### running inference maskrcnn with py2.7

#### error: No module named request

/home/jkuo/work/Mask_RCNN/mrcnn/utils.py in <module>()
---> 20 import urllib.request

urllib is in python3, urllib2 in python2 and does not have request

Solution: comment it out

Ref:
https://stackoverflow.com/questions/24652074/importerror-no-module-named-request

#### error: division by 0 when inference
in python2 divide is integer division, in py3 it is floating division, so we need to make np output to be float
model.py:
line 2456 np.divide
line 2851 tf.divide

utils.py:
line 865 np.divide

visualize.py:
line 66 i/N N is an int cast to float

ref:
https://stackoverflow.com/questions/50590061/different-types-of-divisions-in-tensorflow
https://stackoverflow.com/questions/21316968/division-in-python-2-7-and-3-3

#### catkin_pgk error
error: ImportError: "from catkin_pkg.package import parse_package" failed: No module named catkin_pkg.package

solution:
pip install catkin_pkg

####ethzasl_apriltag2 error
error: libv4l2.h: No such file or directory

#### imgaug
i think it requries python3 as well

### environment that runs demo.py virtualevn setup

### embedding python in c++

ref:
https://www6.software.ibm.com/developerworks/education/l-pythonscript/l-pythonscript-ltr.pdf

#### CMAKE setup example

ref:
https://github.com/ipab-slmc/exotica/blob/master/exotica_python/CMakeLists.txt


### Debug with CMAKE
#### print variable and text

message(STATUS "HERE")
message(STATUS ${PYTHON_INCLUDE_PATH})
message(STATUS ${PYTHON_NUMPY_INCLUDE_CMAKE_PATH})

Ref:
https://cmake.org/cmake/help/v3.0/command/message.html

### pybind11 catkin wrapper
clone this and put it in maplab_dependencies
https://github.com/ipab-slmc/pybind11_catkin

then: catkin build pybind11_catkin

### embedding python in c++ with pybind11

3 problems:

1. find the module path

2. instantiate the maskrcnn and return and save it in memory

3. call the inference method of the maskrcnn object and store the bounding boxes:

example code:
// Calculate e^π in decimal
py::object exp_pi = pi.attr("exp")();
py::print(py::str(exp_pi));

ref:
https://pybind11.readthedocs.io/en/master/advanced/pycpp/object.html

### python version for building pybind11 unreal problem
apparently when you build pybind11 it uses a specifc version of python to build
it can be seen in the  findPythonLibsNew.cmake file
if the python used to build pybind is not the same one you use for running python, you will run into troubles such as segmentation fault when importing tensorflow.

how to make it build against a specific python?
catkin build pybind11_catkin -v --cmake-args -DPYTHON_EXECUTABLE=/home/jkuo/work/Mask_RCNN/venv2.7/bin/python

catkin build pybind11_catkin -v --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python

### For loading images code example
look at interfaces/voxblox_interface/src/integration.cc

### protoc version incompatibility
Failed to load library libsemantify_plugin.so. Error message: libsemantify_plugin.so: cannot open shared object file: No such file or directory
E1017 13:39:47.122388 12953 maplab-console.cc:62] The plugin may not be installed properly. Please try to reinstall the plugin. If the plugin comes from a catkin package, run

solution:
tensorflow 1.8 uses protoc 3.5

download this: https://github.com/protocolbuffers/protobuf/releases/tag/v3.5.0
follow the installation instruction, build from source and sudo make install and sudo ldconfig

### try maplab interface in normal python
import sys
sys.path.append("/home/jkuo/work/Mask_RCNN/samples/maplab_interface")
import maplab_interface
model = maplab_interface.create_model()
import skimage.io


### gdb 

catkin build --no-dep --force-cmake semantify_plugin -v --cmake-args -DCMAKE_BUILD_TYPE=Debug

rosrun --prefix 'gdb --args' maplab_console maplab_console -v=1

gdb --ex run --args  devel/lib/maplab_console/maplab_console

find /usr/ -iname "*type.proto*"

### simple cmake
c++ -O3 -Wall -shared -std=c++11 -fPIC `python3 -m pybind11 --includes` example.cpp -o example`python3-config --extension-suffix`

gcc -O3 -Wall -std=c++11 -fPIC -I /home/jkuo/maplab_ws/devel/include/pybind11_catkin/pybind11 test_py.cc -o test_py -I /usr/bin/python2.7-config

gcc -O3 -Wall -std=c++11 -fPIC -I/usr/include/python2.7 -I/usr/include/x86_64-linux-gnu/python2.7 -I/home/jkuo/maplab_ws/devel/include/pybind11 test_py.cc -o test_py

catkin build test_proj -v --cmake-args -DPYBIND11_PYTHON_VERSION=2.7

g++ test_py.cc -o test_py -std=c++11 -I/usr/include/python2.7 -L/usr/lib/python2.7/config-x86_64-linux-gnu/ -lpython2.7

the problem was the .so file was not in the normal include path, so we use -L to speicify where it is and the -l to specify the name libXXXX
ref:
https://stackoverflow.com/questions/6141147/how-do-i-include-a-path-to-libraries-in-g
https://stackoverflow.com/questions/13440549/gcc-verbose-mode-output-explanation

### upgrade cudnn