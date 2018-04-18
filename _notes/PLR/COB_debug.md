
To set up Caffe for COB

one of the BLVC error:
To get the build to pass on Debian Jessie, I had to (in addition to the above)

1. modify INCLUDE_DIRS in Makefile.config

INCLUDE_DIRS := $(PYTHON_INCLUDE) /usr/local/include /usr/include/hdf5/serial/


2. create symlinks as instructed here (Note the version number after serial)

cd /usr/lib/x86_64-linux-gnu
sudo ln -s libhdf5_serial.so.10.0.2 libhdf5.so
sudo ln -s libhdf5_serial_hl.so.10.1.0 libhdf5_hl.so

Reference:
https://github.com/BVLC/caffe/issues/2347

To setup Caffe with Anaconda
https://gist.github.com/arundasan91/b432cb011d1c45b65222d0fac5f9232c