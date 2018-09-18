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

### File copy with leonhard

#### from leonhard to your pc
scp jkuo@login.leonhard.ethz.ch:/path_to_file/filename .

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

### Train on Leonhard

#### CPU
The conda env maskrcnn contains tensorflow with cpu, so running it only utilizes the cpu
maybe create one with gpu version when needed

#### GPU
training does not require external library such as, imgaug (unless specified in model.train()) or pycocotools,
so we can just use 'load module python_gpu/3.6.4' for the gpu version of the tensorflow

If encounter multiprocessing error:
disable the multiprocessing in model.py, set worker to 1 and use_multiprocessing to False

load the modules:
module load python_gpu/3.6.4 hdf5/1.10.1

Run job, -W min -R requests:
bsub -W 720 -R "rusage[mem=32768,scratch=4096,ngpus_excl_p=1]" python demo.py
bsub -W 1440 -R "rusage[mem=16384,scratch=4096,ngpus_excl_p=1]" python train_hdf5.py
16384
Need to install these packages locally because they are not in the python_gpu module, don't forget to load python
general command:
python -m pip install --user package

package:
scikit-image
opencv-python

#### Stop job
bkill job_id

### Set up Tensorboard

ssh jkuo@login.leonhard.ethz.ch -L 16006:127.0.0.1:16006
What it does is that everything on the port 6006 of the server (in 127.0.0.1:6006) will be forwarded to my machine on the port 16006.

load the cpu module because gpu only works if you submit a job:
module load python_cpu/3.6.4 hdf5/1.10.1

type the command at Mask RCNN root dir:
tensorboard --logdir ./logs --port=16006

On your local machine, go to http://127.0.0.1:16006 and enjoy your remote TensorBoard.


Ref:
https://stackoverflow.com/questions/37987839/how-can-i-run-tensorboard-on-a-remote-server

### Training

### last
When train from last, make sure the name in config is the same as that is how it finds the lastest one. Aslo the training epoch is with respect to the total epoch.
1. change training epoch to the total number of epoch after this training step eg. 100 if starting from 50 with a training set of 50
2. change the dataset index to the correct one

