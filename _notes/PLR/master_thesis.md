### create VImap from bag through rovioli
rosrun rovioli VersaVIS /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1 /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1.bag

cd /media/jkuo/A8DD-8CBF/datasets
rosrun resource_importer import_resources_w_ncamera_yaml.sh asl_small_loop_with_lc_in_mapping_corner_2/raw asl_small_loop_with_lc_in_mapping_corner_2_2019-09-18-16-40-07.bag /VersaVIS/cam0/image_raw ncamera-VersaVIS.yaml asl_small_loop_with_lc_in_mapping_corner_2/with_rgb

### visualize the map
rosrun maplab_console maplab_console
load --map_folder /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1
v

### optimize the map
initialize untrack landmarks so we have all landmarks: itl
optimizes the map with ba num iteration limit: optvi --ba_num_iterations 10
look up what this does: elq
add loop closure constraints: lc
optimize again because lc does not optimize but only add constraints: optvi --ba_num_iterations 10

### import resources
catkin build resource_importer
cd /media/jkuo/7B1C-8EC8/JC_master_thesis/data
rosrun resource_importer import_resources_w_ncamera_yaml.sh quadric-1 quadric-1.bag /VersaVIS/cam0/image_raw /media/jkuo/7B1C-8EC8/JC_master_thesis/data/130deg_fisheye_configuration/ncamera-VersaVIS-cam0.yaml ./quadric-1-color

load --map_folder /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1-color

### Semantify commands
semantify --semantify_start_index 105 --semantify_by_vertex_time true

rosrun netvlad_tf netvlad_node.py
generate_and_save_semantic_object_measurements

rosrun deep_sort_ros deep_sort_node.py
generate_and_save_semantic_object_track_ids_from_deepsort --tracking_confidence_threshold 0.85

visualize_semantic_object_channels_in_visual_frame --semantify_visualization_frequency 0.2 --tracking_confidence_threshold 0.85


rviz
visualize_initialized_semantic_landmarks
visualize_optional_resources_bounding_boxes --semantify_visualization_frequency 0.1
visualize_semantic_object_channels_in_visual_frame --semantify_visualization_frequency 0.1
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantic_landmark_track_id 1
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantic_landmark_track_id 501 --generate_descritpor_filter_by_mask=false

#### descriptor related
ms to show map status and find mission id
generate_descriptor_clusters
display_descriptor_clusters_scores --map_mission
compare_descriptor_clusters_scores --descriptor_comparison_mission_id_0 c277fc00e69fcc150d00000000000000 --descriptor_comparison_mission_id_1 c277fc00e69fcc150d00000000000000 --descriptor_comparison_semantic_landmark_track_ids_0 2,163 --descriptor_comparison_semantic_landmark_track_ids_1 2,163
#### map anchoring
load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
ms

spatially_distribute_missions --spatially_distribute_missions_dimension 2 --spatially_distribute_missions_meters 5
v --vis_color_by_mission 

update_semantic_landmarks_class_ids
visualize_initialized_semantic_landmarks

sbk --map_mission 85965582c59fcc150d00000000000000
ms
now mission 0 T_G_M is set to the reference and becomes known
aam
print_baseframes

## Deep Sort

### Details
requires 3 consecutive hits to have a confirmed track
filters detections by confidence

### running deepsort
python deep_sort_app.py --sequence_dir=./MOT16/MOT16-06 --detection_file=./detections/MOT16-06.npy --min_confidence=0.3 --nn_budget=100 --display=True

### test build
g++ test.cpp -o test -I /usr/include/eigen3

### build Unit Test
catkin run_tests <pkg> --no-deps

cd to the build/ <pkg>
make run_tests tab tab to the test
if want to disable specific unit test write DISABLED_ before the titile of the test in the code

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