### Clone repo
setup the public key following this instruction:
https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/
https://help.github.com/articles/error-permission-denied-publickey/

follow this installation guide:
https://github.com/ethz-asl/maplab_private/wiki/Installation-Ubuntu-16.04

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


### create a map from bag using Rovioli

#### source env
cd maplab_ws/
source devel/setup.bash

#### find flags and help
rosrun maplab_console maplab_console --help | grep -e "-lc_" -A 1

#### check the script 
~/maplab_ws/src/maplab_private/applications/rovioli/scripts/tutorials

it specifies the flags used
Note: images are saved so they can be query later

don't forget to convert the script to executable:
chmod +x my_script

#### run the script
rosrun rovioli my_script /media/jkuo/Data/semester_project/EuRoC/maps/MH_01_easy /media/jkuo/Data/semester_project/EuRoC/bags/MH_01_easy.bag

### visualization

#### download rviz config
https://github.com/ethz-asl/maplab/blob/pre_release_public/july-2018/applications/rovioli/share/rviz-rovioli.rviz

#### command 
rosrun rviz rviz -d /home/jkuo/maplab_ws/src/maplab_private/applications/rovioli/share/
rosrun maplab_console maplab_console

##### in maplab
load --map_folder /media/jkuo/Data/semester_project/EuRoC/maps/MH_01_easy

v (to publish the map in rviz)

### statistics
after selecting a map

type ms to see map statistics
