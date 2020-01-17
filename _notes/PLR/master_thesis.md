### create VImap from bag through rovioli
rosrun rovioli VersaVIS /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1 /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1.bag

cd /media/jkuo/A8DD-8CBF/datasets
rosrun resource_importer import_resources_w_ncamera_yaml.sh asl_small_loop_with_lc_in_mapping_corner_2/raw asl_small_loop_with_lc_in_mapping_corner_2_2019-09-18-16-40-07.bag /VersaVIS/cam0/image_raw ncamera-VersaVIS.yaml asl_small_loop_with_lc_in_mapping_corner_2/with_rgb

rosrun rovioli lidarstick /media/jkuo/A8DD-8CBF/dataset2/asl_koze_table_one_side_with_loop_light_change_max /media/jkuo/A8DD-8CBF/dataset2/asl_koze_table_one_side_with_loop_light_change_max.bag

rosrun rovioli lidarstick /media/jkuo/KINGSTON/dataset2/asl_koze_table_one_side_with_loop_light_change_medium /media/jkuo/KINGSTON/dataset2/asl_koze_table_one_side_with_loop_light_change_medium.bag

### visualize the map
rosrun maplab_console maplab_console
load --map_folder /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1
v

### optimize the map
initialize untrack landmarks so we have all landmarks: itl
optimizes the map with ba num iteration limit: optvi --ba_num_iterations 10
Evaluates and sets the landmark quality of all landmarks: elq 
add loop closure constraints: lc
optimize again because lc does not optimize but only add constraints: optvi --ba_num_iterations 10

### import resources
catkin build resource_importer
cd /media/jkuo/7B1C-8EC8/JC_master_thesis/data
rosrun resource_importer import_resources_w_ncamera_yaml.sh quadric-1 quadric-1.bag /VersaVIS/cam0/image_raw /media/jkuo/7B1C-8EC8/JC_master_thesis/data/130deg_fisheye_configuration/ncamera-VersaVIS-cam0.yaml ./quadric-1-color

load --map_folder /media/jkuo/7B1C-8EC8/JC_master_thesis/data/quadric-1-color

rosrun resource_importer import_resources_w_ncamera_yaml.sh asl_koze_table_one_side_with_loop_light_change_max/raw/ asl_koze_table_one_side_with_loop_light_change_max.bag /VersaVIS/cam0/image_raw ./lidarstick-150-deg-cams-sensors-w-lidar-camera.yaml ./asl_koze_table_one_side_with_loop_light_change_max/with_rgb/

rosrun resource_importer import_resources_w_ncamera_yaml.sh asl_koze_table_one_side_with_loop_light_change_max/raw/ adjusted.bag /VersaVIS/cam0/image_raw ./lidarstick-150-deg-cams-sensors-w-lidar-camera.yaml ./asl_koze_table_one_side_with_loop_light_change_max/with_rgb/

### Semantify commands
rosrun mask_rcnn_ros mask_rcnn_node.py
semantify --semantify_start_index 0 --semantify_by_vertex_time true

rosrun netvlad_tf netvlad_node.py
generate_and_save_semantic_object_measurements

rosrun deep_sort_ros deep_sort_node.py
generate_and_save_semantic_object_track_ids_from_deepsort --tracking_confidence_threshold 0.85

visualize_semantic_object_channels_in_visual_frame --semantify_visualization_frequency 0.2 --tracking_confidence_threshold 0.85

save --map_folder path/to/save/the/map --overwrite

rviz
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
visualize_optional_resources_bounding_boxes --semantify_visualization_frequency 0.1
visualize_semantic_object_channels_in_visual_frame --semantify_visualization_frequency 0.1
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantic_landmark_track_id 1
evaluate_semantic_landmark_with_track_id --semantify_visualization_frequency 0.1 --semantic_landmark_track_id 501 --generate_descritpor_filter_by_mask=false
evaluate_semantic_landmark_with_track_id --map_mission c277fc00e69fcc150d00000000000000 --semantic_landmark_track_id 89
evaluate_semantic_landmark_with_track_id --semantic_landmark_track_id 89

### create semantic landmarks
itsl --vi_map_semantic_landmark_quality_min_observation_angle_deg 10

#### match landmarks in a map
ms to show map status and find mission id
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
match_semantic_landmarks_in_one_mission --map_mission 85965582c59fcc150d00000000000000
display_descriptor_clusters_scores --map_mission
compare_descriptor_clusters_scores --descriptor_comparison_mission_id_0 c277fc00e69fcc150d00000000000000 --descriptor_comparison_mission_id_1 c277fc00e69fcc150d00000000000000 --descriptor_comparison_semantic_landmark_track_ids_0 2,163 --descriptor_comparison_semantic_landmark_track_ids_1 2,163

#### traditional map anchoring
load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
ms

spatially_distribute_missions --spatially_distribute_missions_dimension 2 --spatially_distribute_missions_meters 5
v --vis_color_by_mission 

update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map

sbk --map_mission 85965582c59fcc150d00000000000000
ms
now mission 0 T_G_M is set to the reference and becomes known
aam
print_baseframes

#### semantic landmark map anchoring
load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
ms
sbk --map_mission 85965582c59fcc150d00000000000000
ms
spatially_distribute_missions --spatially_distribute_missions_dimension 2 --spatially_distribute_missions_meters 5
v --vis_color_by_mission
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
anchor_mission_with_semantic_landmarks --semantic_landmark_max_match_candidate_distance 8 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.3

## traditional map anchoring
### create database
addAllMissionsWithKnownBaseFrameToProvidedLoopDetector()
adds all mission with known frames to the loop detector database, where the images and landmarks 
are compressed to descriptors and saved in the inverted index structure using loop_detector->addMissionToDatabase()

### Find loop closure between the specified mission_id and the database
in probeMissionAnchoring()

loop_detector.detectLoopClosuresMissionToDatabase(
      mission_id, kMergeLandmarksOnProbe, kAddLoopclosureEdgesOnProbe,
      &result->num_vertex_candidate_links,
      &result->average_landmark_match_inlier_ratio, map, &result->T_G_M,
      &inlier_constraints);

in loop-dectector-node.cc:
### query for matches
detectLoopClosuresVerticesToDatabase():
  for every vertex in the specified mission:
    queryVertexInDatabase():
      converts all frames in the vertex to projected image
      query the matches for the projected images in database
        covisibility check:
          From the paper Improving Image-Based Localization by Active Correspondence Search
          Incorporating Additional Visibility Information section:
          a) filter 3d points: when making a query with an image to the database, you can get 2d-3d correspondences that are incorrect,
                               but matched becauset of descriptor similarity, we wish to remove these as they introduce problems for RANSAC.
                               We cannot remove by thresholding distance from the 3d points with close proximity can be generated from 
                               different views. The paper said it only keeps 3d points they are two edges away in the bipartite graph, 
                               which means it only keeps the 3d points that are also visible in the frames that contains the query point.
          b) RANSAC prefiltering: from the remaining correspondences, we do a ransac and filters out outliers and obtain a camera pose.
          c) using camera sets: try to merge cameras to find more continuous point visibility because the previous method could be too stringent.
                                From the set of similar images of the query image, we select k closest cameras that have less than 60 deg view 
                                point difference from the query image and form a set S. A minimal subset S' ⊂ S ={sim(Ij )} of the camera sets 
                                is selected such that every image Ij is contained in at least one set s ∈ S'. This is solved using greedy 
                                algorithm.
      for each matching frame:
        convertFrameMatchesToConstraint():
          struct FrameKeyPointToStructureMatch {
            vi_map::KeypointIdentifier keypoint_id_query;
            vi_map::VisualFrameIdentifier keyframe_id_result;
            vi_map::LandmarkId landmark_result; }
          each matches for this frame is converted to a constraint and push_back to raw_constraint
      the raw constraints are send to handleLoopClosures
      loop_closure_handler::LoopClosureHandler handler.handleLoopClosures():
        for each raw constraint structure match (keypoint to landmark match):
          fill in the the measurements and G_landmark_positions matrix with the size of the total num raw constraints
        query_vertex_id is the vertex that we checked against the database and obtained matches for
        use RANSAC to compute the transformation of this vertex to the map and gets T_G_I_ransac
          pose_estimator.absoluteMultiPoseRansacPinholeCam(measurements, G_landmark_positions):
            returns T_G_I_ransac and the set of inlier constraints
        getBestStructureMatchForEveryKeypoint():
          for each keypoint, associate it with the matches in inlier set that has the lowest reprojection error
        LoopClosureHandler::mergeLandmarks():
          map_->mergeLandmarks(query_landmark_to_be_deleted, map_landmark):
            take all the observation of query_landmark_to_be_deleted and add it to map_landmark's observations
            set all vertices that observe query_landmark_to_be_deleted to observe map_landmark
            delete query_landmark_to_be_deleted from book keeping
        Now we need to find the vertex in the pose graph that we create an edge with.
        We choose the vertex that has the most overlapping landmarks from the inlier structure matches
        lc_edge_target_vertex_id = vi_map_helpers::getVertexIdWithMostOverlappingLandmarks(query_vertex_id, *inlier_structure_matches, *map_, &commonly_observed_landmarks);
        if (add_loopclosure_edges):
          loop_closure_handler::addLoopClosureEdge(query_vertex_id, commonly_observed_landmarks,lc_edge_target_vertex_id, *T_G_I_ransac, map_):

      T_M_I = query_vertex.get_T_M_I();
      T_G_M2 = T_G_I_ransac * T_M_I.inverse();
      Then adds the T_G_M2 to the list T_G_M2_vector_local
  After we collect all the estimated T_G_M from all the verticies, we do a transformationRansac()
  to find the inlier within the set and we use that as the transformation T_G_M_estimate for the specified mission
  There is a kNumInliersThreshold check after the transformation RANSAC and it returns empty T_G_M_estimate if failed
  What is Transformation RANSAC?
    It uses the following two param to check 
    lc_mission_baseframe_ransac_max_orientation_error_rad, lc_mission_baseframe_ransac_max_position_error_m to check if 
    other T_G_M2_vector_local are inliers of the selected T_G_M2_vector_local.
    After the num_iteration is finished, it perform a least square refinement on the best set of inliers

## traditional loop closure
### loop closure plugin
creates a map merger  VIMapMerger
then merger.findLoopClosuresBetweenAllMissions()
in merger.findLoopClosuresBetweenMissions(mission_ids):
creates/load a loop detector (database)
if create:
do the top half triangle of the comparison matrix including self 
loop_detector.addMissionToDatabase(*it, *map_)
  loop_detector.detectLoopClosuresAndMergeLandmarks(mission):
    detectLoopClosuresMissionToDatabase():
      get all vertices in mission
      detectLoopClosuresVerticesToDatabase():
        queryVertexInDatabase()


### semantic landmark loop closure
load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
update_semantic_landmarks_class_ids 
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map --semantic_landmark_class_filter "1,73,61,57"
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c3c402890f1ec6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 100 --semantic_landmark_lc_extend_visible_verticies=true --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_add_edge_between_topological_center_vertices=true
optvi --ba_visualize_every_n_iterations 1
rtsl
v
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map

#### Tuning parameters for semantic loop closure:

##### view point change:
load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
v
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c3c402890f1ec6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 100 --semantic_landmark_lc_extend_visible_verticies=true --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_num=4 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.6 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
visualize_semantic_loop_closure_edge_covariances

load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_round_and_round_he_goes/with_optvi_nolc_semantic_landmarks
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c9b4d48ec021c6150d00000000000000 --loop_closure_mission_id_source c9b4d48ec021c6150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 40 --semantic_landmark_lc_extend_visible_verticies=true --semantic_landmark_anchoring_ransac_min_inlier_num=5 --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.8 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false 
visualize_semantic_loop_closure_edge_covariances

load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 7c72ec38581ec6150d00000000000000 --loop_closure_mission_id_source 7c72ec38581ec6150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 60 --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_num=5 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.5 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
visualize_semantic_loop_closure_edge_covariances

loop closure between two maps:

load --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-1/quadric_2_1_with_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/old_sensor_setup/data/quadric-2-2/quadric_2_2_with_semantic_landmarks/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 85965582c59fcc150d00000000000000 --loop_closure_mission_id_source c277fc00e69fcc150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 100 --visualize_accepted_loop_closure_edge=false --semantic_landmark_anchoring_ransac_min_inlier_num=8 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.2 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
evaluate_semantic_landmark_with_track_id --map_mission 85965582c59fcc150d00000000000000 --semantic_landmark_track_id 2
evaluate_semantic_landmark_with_track_id --map_mission c277fc00e69fcc150d00000000000000 --semantic_landmark_track_id 1


load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
sbk --map_mission 7c72ec38581ec6150d00000000000000
anchor_mission_with_semantic_landmarks --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.15
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 7c72ec38581ec6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 60 --semantic_landmark_max_match_candidate_distance 8 --visualize_accepted_loop_closure_edge=false --semantic_landmark_anchoring_ransac_min_inlier_num=6 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.2 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
evaluate_semantic_landmark_with_track_id --map_mission c3c402890f1ec6150d00000000000000 --semantic_landmark_track_id 415
evaluate_semantic_landmark_with_track_id --map_mission 7c72ec38581ec6150d00000000000000 --semantic_landmark_track_id 409

load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_round_and_round_he_goes/visual_lc_with_semantic_lm
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref c9b4d48ec021c6150d00000000000000 --loop_closure_mission_id_source c3c402890f1ec6150d00000000000000 --semantic_landmark_lc_extend_visible_verticies_num 80 --semantic_landmark_max_match_candidate_distance 5 --visualize_accepted_loop_closure_edge=false --semantic_landmark_anchoring_ransac_min_inlier_num=4 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.3 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false
evaluate_semantic_landmark_with_track_id --map_mission c3c402890f1ec6150d00000000000000 --semantic_landmark_track_id 853
evaluate_semantic_landmark_with_track_id --map_mission c9b4d48ec021c6150d00000000000000 --semantic_landmark_track_id 571
optvi --map_mission c3c402890f1ec6150d00000000000000
optvi --map_mission c9b4d48ec021c6150d00000000000000

create groundtruth from all maps:
load --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_one_side_with_loop/with_optvi_nolc_semantic_landmarks/
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_round_and_round_he_goes/with_optvi_nolc_semantic_landmarks
load_merge_map --map_folder /media/jkuo/A8DD-8CBF/datasets/asl_koze_table_both_sides_with_loop/with_semantic_landmarks20/
lc
optvi

##### light intensity change:
for this map, we ignore chairs because we had to reduce the minimum view angle required to get more landmarks on the table.
As a consequence, more bad chair landmarks appear made the inlier ratio kind of useless. So we ignore chairs for this map.

load --map_folder /media/jkuo/A8DD-8CBF/dataset2/asl_koze_table_one_side_with_loop_light_change_medium/with_semantic_landmarks5_point7confidence/
update_semantic_landmarks_class_ids
visualize_semantic_landmarks_and_generate_track_id_to_semantic_landmark_map --semantic_landmark_class_filter "1,73,61,57"
generate_descriptor_clusters
calculate_semantic_landmark_covariances
loop_close_missions_with_semantic_landmarks --loop_closure_mission_id_ref 093e162baf13df150d00000000000000 --loop_closure_mission_id_source 093e162baf13df150d00000000000000 --semantic_landmark_matching_filter_by_match_candidate_distance=true --semantic_landmark_lc_extend_visible_verticies_num 100 --semantic_landmark_lc_extend_visible_verticies=true --visualize_accepted_loop_closure_edge=true --semantic_landmark_anchoring_ransac_min_inlier_num=6 --semantic_landmark_anchoring_ransac_min_inlier_ratio 0.6 --semantic_landmark_lc_max_covisible_object_candidate_distance_difference 0.6 --semantic_landmark_lc_add_edge_between_topological_center_vertices=false --semantic_landmark_lc_merge_matched_landmarks=false --semantic_landmark_lc_remove_old_edges=false

load --map_folder /media/jkuo/A8DD-8CBF/dataset2/asl_koze_table_one_side_with_loop_light_change_max/with_semantic_landmarks20/
mission id 2e0aa9515913df150d00000000000000

load --map_folder /media/jkuo/A8DD-8CBF/dataset2/asl_koze_table_both_side_with_loop_light_change_medium/with_semantic_landmarks5_point6/
mission id 80d309e60614df150d00000000000000

load --map_folder /media/jkuo/KINGSTON/dataset2/asl_koze_table_one_side_with_loop_light_change_medium/with_semantify
mission id 80d309e60614df150d00000000000000

## export posegraph in rpg format for evaluation
ettc_rpg --pose_export_file ~/semantic_lc_top_OneSide.txt
ettc_rpg --map_mission 093e162baf13df150d00000000000000 --pose_export_file ~/gt_light_change_medium_OneSide.txt
ettc_rpg --map_mission 2e0aa9515913df150d00000000000000 --pose_export_file ~/gt_light_change_max_OneSide.txt
ettc_rpg --map_mission 80d309e60614df150d00000000000000 --pose_export_file ~/gt_light_change_medium_BothSide.txt

## traditional OptimizerPlugin::optimizeVisualInertial
### optvi
map-structure/posegraph
the work is based on 

## trajectory evaluation
rosrun rpg_trajectory_evaluation analyze_trajectories.py asl_map_lc_comparison.yaml --output_dir=/home/jkuo/maplab_ws/src/rpg_trajectory_evaluation/evaluation/asl_map_lc_comparison --results_dir=/home/jkuo/maplab_ws/src/rpg_trajectory_evaluation/results/asl_map --platform laptop --odometry_error_per_dataset --plot_trajectories --rmse_table --rmse_boxplot

rosrun rpg_trajectory_evaluation analyze_trajectory_single.py .


## dataset storage
sftp j225-students@data.asl.ethz.ch

Location:
/J225-students/jkuo-semantic-mapping

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