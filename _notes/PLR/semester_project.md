### Setting up matterport maskrcnn with gpu and virtualenv
 git clone https://github.com/matterport/Mask_RCNN.git

### Run sample
jupyter notebook in command line then open sample.ipynb

####create a virtual environment
virtualenv --python=python3.5 venv3.5

#### activate and deactivate virtualenv
to activate: source venv3.5/bin/activate

### Errors
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

