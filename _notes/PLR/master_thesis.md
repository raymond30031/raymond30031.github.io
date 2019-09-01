### create VImap from bag through rovioli
rosrun rovioli VersaVIS /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1 /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1.bag

### visualize the map
rosrun maplab_console maplab_console
load --map_folder /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1
v

### import resources
catkin build resource_importer
cd /media/jkuo/7B1C-8EC8/JC_master_thesis/data
rosrun resource_importer import_resources_w_ncamera_yaml.sh quadric-1 quadric-1.bag /VersaVIS/cam0/image_raw /media/jkuo/7B1C-8EC8/JC_master_thesis/data/130deg_fisheye_configuration/ncamera-VersaVIS-cam0.yaml ./quadric-1-color

load --map_folder /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1-color

## Deep Sort

### Details
requires 3 consecutive hits to have a confirmed track

### running deepsort
python deep_sort_app.py --sequence_dir=./MOT16/MOT16-06 --detection_file=./detections/MOT16-06.npy --min_confidence=0.3 --nn_budget=100 --display=True

### test build
g++ test.cpp -o test -I /usr/include/eigen3

### build Unit Test
catkin run_tests <pkg> --no-deps

### Why interpolate pose in landmark triangulation?
IMU has higher rate and frames are taken at a slower rate. 
They interpolate the frame pose from imu message using frame timestamp because the two can be out of sync.

## Map-Structure

### vi-map

#### landmark index
it is an unordered map that saves the relationship between landmark id and the vertex that stores the landmark.

#### vertex

#### landmarks
landmark has a identifier list that stores the frame id and the measurement index in that frame which corresponds to this landmark

### vi-map testing
there is a thing called vi-map generator, map structure info are specified first and generated after calling generateMap();


##c++ tips
### copy vector to eigen::vector
https://stackoverflow.com/questions/17036818/initialise-eigenvector-with-stdvector
### copy eigen to std::vector
https://stackoverflow.com/questions/54682743/how-to-map-a-eigenmatrix-to-stdvectoreigenvector