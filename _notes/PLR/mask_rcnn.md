### Clone repo

### environment setup with conda

Want to use pip install for a conda env?
1. Create your virtual environment with 'conda create --name virtual_env_name' , replacing ‘virtual_env_name’ with the name of your virtual environment
2. Switch to your virtual environment with 'source activate virtual_env_name' , again replacing ‘virtual_env_name’ with the name of your virtual environment
3. Run 'conda install pip' , which will install pip to your virtual environment directory
4. Do '/anaconda/envs/venv_name/bin/pip install package_name'
Ref:
http://www.puzzlr.org/install-packages-pip-conda-environment/

For installing environment using requirement.txt with conda:
conda install --yes --file /path_to_file 

Need to install opencv before imgaug because imgaug depends on it.

missing opencv:
pip install opencv-python

Ref:
https://stackoverflow.com/questions/19876079/opencv-cannot-find-module-cv2

missing imgaug:
pip install git+https://github.com/aleju/imgaug

Ref: https://github.com/aleju/imgaug

Still throws error because opencv-python and imgaug are not in the default channels of conda for searching.
So I removed them and install the requirement.txt again.

### Run demo from leonhard

1. first conda install jupyter
2. login leonlard using ssh user@login.leonhard.ethz.ch -L 8888:127.0.0.1:8888
3. jupyter notebook
It will complain about no web browser, just copy and paste the http link to a browser on your computer.

### Run demo in sample folder

Error: no module pycocotool
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