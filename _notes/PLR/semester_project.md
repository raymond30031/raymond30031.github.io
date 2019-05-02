###build
catkin clean
catkin build maplab
catkin build --no-deps semantify_plugin
catkin build --no-deps mask_rcnn_ros
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

#### using service in mask_rcnn_ros_node
Apparently, for keras, you get the following error:
Tensor Tensor("mrcnn_detection/Reshape_1:0", shape=(1, 100, 6), dtype=float32) is not an element of this graph.
When calling inference from another process.

see solution and reference:
https://blog.csdn.net/Cyril__Li/article/details/79054596 

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

### ZED MINI
#### installation

Download:
https://www.stereolabs.com/developers/release/2.7/#sdkdownloads_anchor
Documentation:
https://docs.stereolabs.com/overview/getting-started/installation/
Git Driver:
https://github.com/stereolabs/zed-ros-wrapper

#### Roslaunch wrapper
once driver is installed:
roslaunch zed_wrapper zed.launch
roslaunch zed_display_rviz display_zedm.launch # open a ZED Mini

cd ~/maplab_ws/src/maplab_private/applications/rovioli/share/zed_mini
./run_zedmini_rostopic.sh ~/save_folder
./run_zedmini_rosbag.sh /media/jkuo/ASL_SamsungT5/zed_mini/map/ros_time/07/map /media/jkuo/ASL_SamsungT5/zed_mini/map/ros_time/07/zed_mini_map_2018-11-20-10-47-59.bag

rosrun rviz rviz -d /home/jkuo/maplab_ws/src/maplab_private/applications/rovioli/share/

record bag:
rosbag record -o zed_mini /rosout /rosout_agg /zed/imu/data /zed/imu/data_raw /zed/left/camera_info /zed/left/camera_info_raw /zed/left/image_raw_color /zed/right/camera_info /zed/right/camera_info_raw /zed/right/image_raw_color

### Optimize vimap with maplab
itl: Initialize all unprocessed but tracked landmarks. relaxes the pose graph 
rtl: retriangulate all landmarks
repeat:
  optvi: optimize pose graph based on visual and imu
  elq: evaluate landmarks quality, remove bad quality landmarks
  lc: loop closure, merge points
  optvi:
  elq:


### Kalibr
MultiCamera:
kalibr_calibrate_cameras --target aprilgrid.yaml --bag /home/jkuo/zed_mini_both_current_sub.bag --models pinhole-equi pinhole-equi --topics /zed/left/image_raw_color /zed/right/image_raw_color  --verbose --show-extraction

IMU Camera calibration:
kalibr_calibrate_imu_camera --bag /home/jkuo/zed_mini_both_current.bag --cam ./camchain-homejkuozed_mini_both_current.yaml --imu imu.yaml --target aprilgrid.yaml

#### calibrate multi camera
rosrun kalibr_calibrate_cameras --target aprilgrid.yaml --bag /home/jkuo/zed_mini_both_current_2018-11-07-14-30-50.bag --models pinhole-equi pinhole-equi --topics /zed/left/image_raw_color /zed/right/image_raw_color

#### convert calibration result for rovioli use
kalibr_maplab_config --cam camchain-imucam-.zed_mini_imu.yaml --imu imu-.zed_mini_imu.yaml
ref:
https://github.com/ethz-asl/maplab_private/wiki/Calibration

#### Errors
##### cannot import name NavigationToolbar2Wx
solution:
https://github.com/ethz-asl/kalibr/issues/202

install igraph
https://stackoverflow.com/questions/36200707/error-with-igraph-library-deprecated-library


###Backup
rovio has some deeper shit below that does not accet colour images. Adding the following code does not make rovioli run,
but makes it stop complaining about cvtColor.

making colour image work:
diff --git a/include/rovio/ImgUpdate.hpp b/include/rovio/ImgUpdate.hpp
index d7852b3..2784a22 100644
--- a/include/rovio/ImgUpdate.hpp
+++ b/include/rovio/ImgUpdate.hpp
@@ -614,7 +614,11 @@ ImgOutlierDetection<typename FILTERSTATE::mtState>,false>{
     CHECK(filterState.t_ == meas.aux().imgTime_);
     for(int i=0;i<mtState::nCam_;i++){
       if(doFrameVisualisation_){
-        cvtColor(meas.aux().pyr_[i].imgs_[0], filterState.img_[i], CV_GRAY2RGB);
+        int img_channel = (meas.aux().pyr_[i].imgs_[0]).channels();
+        if(img_channel == 3)
+          continue;
+        else
+          cvtColor(meas.aux().pyr_[i].imgs_[0], filterState.img_[i], CV_GRAY2RGB);

+++ b/applications/rovioli/include/rovioli/ros-helpers.h
@@ -18,6 +18,11 @@
 
 #include <vio-common/vio-types.h>
 
+//for converting bgra to bgr
+#include <opencv2/imgproc.hpp>
+#include <opencv2/highgui/highgui.hpp>
+#include <opencv2/core.hpp>
+
 namespace rovioli {
 
 inline int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
@@ -37,13 +42,55 @@ inline vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
   return imu_measurement;
 }
 
+/* to switch on type string from image encoding
+   avoid using if for better optimization
+   discussion:
+   https://stackoverflow.com/questions/650162/why-the-switch-statement-cannot-be-applied-on-strings
+*/
+enum EncodingStringValue { evMono8,
+                           evBGR8,
+                           evBGRA8};
+
+// Map to associate the strings with the enum values
+static std::map<std::string, EncodingStringValue> mapStringValues = {
+  {"mono8", evMono8},
+  {"bgr8", evBGR8},
+  {"bgra8", evBGRA8}}; 
+
 inline vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
     const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
   CHECK(image_message);
+  // LOG(INFO) << "img ecnoding " << image_message->encoding;
+  // LOG(INFO) << "img height " << image_message->height;
+  // LOG(INFO) << "img width " << image_message->width;
+  // LOG(INFO) << "img step " << image_message->step;
+  // LOG(INFO) << "img array length " << sizeof(image_message->data);///sizeof(image_message->data);
+  //check for conversion
+  bool cvtBGRA2BGR = false;
   cv_bridge::CvImageConstPtr cv_ptr;
   try {
-    cv_ptr = cv_bridge::toCvShare(
-        image_message, sensor_msgs::image_encodings::TYPE_8UC1);
+    // to support different encodings
+    switch(mapStringValues[image_message->encoding]){
+      case evMono8:
+        cv_ptr = cv_bridge::toCvShare(
+          image_message, sensor_msgs::image_encodings::TYPE_8UC1);
+        break;
+      case evBGR8:
+        cv_ptr = cv_bridge::toCvShare(
+          image_message, sensor_msgs::image_encodings::TYPE_8UC3);
+        break;
+      case evBGRA8:
+        cv_ptr = cv_bridge::toCvShare(
+          image_message, sensor_msgs::image_encodings::TYPE_8UC4);
+        cvtBGRA2BGR = true;
+        break;
+      default:
+        LOG(INFO) << "using default for unhandled image encoding: " 
+                  <<image_message->encoding;
+        cv_ptr = cv_bridge::toCvShare(
+          image_message, sensor_msgs::image_encodings::TYPE_8UC1);
+    }
+    
   } catch (const cv_bridge::Exception& e) {  // NOLINT
     LOG(FATAL) << "cv_bridge exception: " << e.what();
   }
@@ -55,6 +102,23 @@ inline vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
   // data alive without holding on to the CvImage is not possible without
   // letting all the code depend on ROS and boost.
   image_measurement->image = cv_ptr->image.clone();
+  // convert from bgra to bgr
+  if(cvtBGRA2BGR) {
+    int rows = image_measurement->image.rows;
+    int cols = image_measurement->image.cols;
+    cv::Mat dst(rows,cols,CV_8UC3);
+    LOG(INFO) << "old img channel: "<<(image_measurement->image).channels();
+    LOG(INFO) << "old img rows: "<<rows;
+    LOG(INFO) << "old img cols: "<<cols;
+    cv::imwrite( "/home/jkuo/old.jpg", image_measurement->image );
+    cv::cvtColor(image_measurement->image, dst, cv::COLOR_BGRA2BGR);
+    image_measurement->image = dst;
+    cv::imwrite( "/home/jkuo/new.jpg", image_measurement->image );
+    LOG(INFO) << "new img channel: "<<(image_measurement->image).channels();
+    LOG(INFO) << "new img rows: "<<image_measurement->image.rows;
+    LOG(INFO) << "new img cols: "<<image_measurement->image.cols;
+  }
+  


### ROSBAG
record bag for a map:
rosbag record -o zed_mini_map /rosout /rosout_agg /zed/imu/data /zed/imu/data_raw /zed/left/camera_info /zed/left/camera_info_raw /zed/left/image_raw_color /zed/right/camera_info /zed/right/camera_info_raw /zed/right/image_raw_color --buffsize=0 --lz4 --chunksize=4096
#### split bag
rosbag filter zed_mini_2018-11-08-14-00-29.bag output.bag "t.secs <= 1541682031.07"

### resource importer
need to catkin build resource_importer
rosrun resource_importer import_resources_w_ncamera_yaml.sh map_folder_0 output.bag /zed/left/image_raw_color d90edaf392d0e4d2e1791e4f8f7be49e.yaml ./map_resource

### Start semantify
rosrun mask_rcnn_ros mask_rcnn_node.py
rosrun maplab_console maplab_console
load --map_folder /media/jkuo/ASL_SamsungT5/zed_mini/map/ros_time/01/map_resource/
semantify
semantify --semantify_start_index 97
res_stats
save --map_folder path/to/save/the/map --overwrite

### Visualize Bounding boxes
visualize_bounding_boxes --vis_resource_visualization_frequency 1

### Generate Quadrics
generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 15
eval_quadric --eval_quadric_id 180 --eval_bb_up_to_num 20

#### no effect for centroid_keypoints_shift ratio
##### for multichair case:
generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 15 --init_quadric_min_bounding_box_num 50 --semantify_visualization_frequency 0.1 --tracking_centroid_keypoint_shift_ratio 100

generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 20 --init_quadric_min_bounding_box_num 40 --semantify_visualization_frequency 0.05 --tracking_centroid_keypoint_shift_ratio 100.0

generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 10 --init_quadric_min_bounding_box_num 20 --semantify_visualization_frequency 0.05 --tracking_centroid_keypoint_shift_ratio 100.0

to disable condition number check:
visualize_initialized_tracked_objs --init_quadric_max_condition_number 99999

eval_quadrics_quality --eval_quadrics_field_of_view 45

trial #1:
generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 12 --init_quadric_min_bounding_box_num 15 --semantify_visualization_frequency 0.05 --tracking_centroid_keypoint_shift_ratio 100.0 --init_quadric_max_condition_number 99999

visualize_initialized_tracked_objs --init_quadric_max_condition_number 3000 --init_quadric_min_bounding_box_num 15 --semantify_visualization_by_class true

eval_quadrics_quality --eval_quadrics_field_of_view 43

trial #2:
generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 12 --init_quadric_min_bounding_box_num 10 --semantify_visualization_frequency 0.05 --tracking_centroid_keypoint_shift_ratio 10.0 --init_quadric_max_condition_number 3000

visualize_initialized_tracked_objs --init_quadric_max_condition_number 3000 --init_quadric_min_bounding_box_num 10 --semantify_visualization_by_class true

eval_quadrics_quality --eval_quadrics_field_of_view 40

eval_quadric --eval_quadric_id 588 --eval_bb_up_to_num 0 --semantify_visualization_frequency 0.05 --init_quadric_max_condition_number 3000 --init_quadric_min_bounding_box_num 10 --semantify_visualization_by_class true

For report:
generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 12 --init_quadric_min_bounding_box_num 10 --semantify_visualization_frequency 0.05 --tracking_centroid_keypoint_shift_ratio 10.0 --init_quadric_max_condition_number 99999 --semantify_visualization_by_class true

visualize_initialized_tracked_objs --init_quadric_max_condition_number 3000 --init_quadric_min_bounding_box_num 10 --semantify_visualization_by_class true

eval_quadrics_quality --eval_quadrics_field_of_view 40

##### for the single chair case:
generate_init_quadric --tracking_method 1 --tracking_min_persistent_keypoints_threshold 2 --init_quadric_min_bounding_box_num 40 --semantify_visualization_frequency 0.05 --tracking_centroid_keypoint_shift_ratio 100.0

perfect data association:
generate_init_quadric --tracking_method 0 --semantify_visualization_frequency 0.05 --tracking_centroid_threshold 30

### Eval/visualize quadric id
eval_quadric --eval_quadric_id 0 --eval_bb_up_to_num 0 --semantify_visualization_frequency 0.05

### Eval quadrics quality, remove ones that has limited field of view
eval_quadrics_quality --eval_quadrics_field_of_view 30

## Math
### Quadric to Conic
General Ellipse equation:
https://www.maa.org/external_archive/joma/Volume8/Kalman/General.html

The opencv function for drawing ellipse takes some other parameters:
https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html#ellipse

to draw the full ellipse, set start angle to 0 and end angle to 360
The axes represents the major and minor axis of the ellipse.
https://books.google.ch/books?id=LPm3DQAAQBAJ&pg=PA160&lpg=PA160&dq=opencv+size+axes&source=bl&ots=2vLlTbfiwf&sig=BhsCpLYzhrF3gENrqAbNIQb-77w&hl=en&sa=X&ved=2ahUKEwiUheSIv_TeAhXCAewKHVybClEQ6AEwBXoECAQQAQ#v=onepage&q=opencv%20size%20axes&f=false
example:
http://opencvexamples.blogspot.com/2013/10/basic-drawing-examples.html

To get the axis length:
https://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections
https://math.stackexchange.com/questions/616645/determining-the-major-minor-axes-of-an-ellipse-from-general-form

To get the center of the ellipse and the rotation angle:
https://math.stackexchange.com/questions/2766028/i-want-to-draw-ellipse-using-ellipse-equation-general

More math reference:
https://www.maa.org/external_archive/joma/Volume8/Kalman/TransformedEqn.html
https://www.maa.org/external_archive/joma/Volume8/Kalman/General.html

### inspect image pixel value
edisplay --gl ~/mask_inverse.ppm

### OpenCV questions:
#### OpenCV Point(x,y) represent (column,row) or (row,column)
So, this means that src.at(i,j) is using (i,j) as (row,column) but Point(x,y) is using (x,y) as (column,row)
Ref: https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column