### Installation
follow the instructions in the rpg_svo_pro repo readme

### Build
catkin build

### run test
roslaunch svo_ros run_from_bag.launch cam_name:=svo_test_pinhole

rosbag play /media/jkuo/Data/side_project/airground_rig_s3_2013-03-18_21-38-48.bag

set the namespace to svo, then it would show # of tracked feature

### pipeline
svo_node (wraps a svo_interface_) -> svo_interface sets up the publisher and subscriber -> creates the pipeline type(mone/stereo param to constructor) -> new thread for monoLoop -> set up subscriber for monoCallBack()

In monoCallBack():
  //process image bundle
  processImageBundle(): 
    svo_->addImageBundle(images, timestamp_nanoseconds)
    note svo_ is a framehandler mono derived from base 

In framehandler_base/mono:
  in addImageBundle() from base class:
    puts the cv:Mat imgs into framebundle and calls addFrameBundle() from base class
  in addFrameBundle from base class:
    assign the framebundle to new_frames_ (current framebundle being processed)
    res = processFrameBundle() from mono: // Perform tracking.
      processFrame() or processFirstFrame() from mono:
        // processFirstFrame until initialized
        processFirstFrame():
          sets first frame as keyframe
          adds first frame as keyframe in map
        // only go into processFrame once map is initialized by two keyframe
        processFrame():
          sparseImageAlignment():
          projectMapInFrame()
          poseOptimization():
          // adds our entropy calculation after pose opt, so can be used later in kf selection
          keyframeSelection():
          if keyframe:
            add keyframe to map
          if map size > max:
            remove the furthest keyframe from current pose
    assign the new_frames_ to last_frames (last framebundle that was processed)

In framehandler_stereo:
  ProcessFrameBundle():
    if stage_ == initialization:
      make both frames from cameras keyframes and triangulate landmarks
    if stage_ == tracking:
      ProcessFrame():
        SparseImageAlignment()
        projectMapInFrame()
        PoseOptimization()
          Here if not enough features then we just make a new keyframe by triangulating landmarks.
          It does not re-initialize using the initializer, so it does not lose the current map in the pipline
        keyframeSelection()
        MapCulling()
  MakeKeyframe():
    only the frame from cam0 is saved in map and used as keyframe because Christian assumed the cameras are close enough,
    so the landmarks they observe have a large overlap and it is not necessary to save both

projectMapInFrame() projects the current map(consists of keyframes) into the current frame and check if the keyframes has 
overlapping views and returns matched features.

FrameHandlerBase::projectMapInFrame():
  // get closest keyframes for every frame in the current framebundle
  overlap_kfs_.clear() for every new framebundle
  Map::getClosestNKeyframesWithOberlap():
    Check all keyframes in the map if their keypoints are visible in the current framebundle. If one keypoint is visible,
    then push the FramePtr of the keyframe into a close_kfs list. It stores the FramePtr and the distance of the keyframe
    from the current frame. The list is then sorted based on distance with closest first and furthest last. It is then
    truncated to N size long, where N = std::min(num_frames, overlap_kfs.size()) and num_frames is option.max_n_kf.
    Now we have the overlap_kfs_
  // reprojection by reprojectors
  Reprojector::reprojectFrames():
    reproject all landmarks in overlapping keyframes to current frame. These patches of these landmarks become candidates 
    for matching to patches of the reprojection of these landmarks in the current frame. This is the Relaxation and Refinement
    part of the paper.
    In the end, the func returns # of matched features (corners and edgelets)

   
keyframeSelection() is the needNewKf() function in frame_handler_base, it is type casted into NewKeyFrameCriteria need_new_kf_
  for stereo, there is only forward mode
  Forward:
    if(n_tracked_fts > options_.kfselect_numkfs_upper_thresh)
    {
      VLOG(40) << "KF Select: NO NEW KEYFRAME Above upper bound";
      return false;
    }

    // TODO: this only works for mono!
    if(last_frames_->at(0)->id() - map_->last_added_kf_id_ < options_.kfselect_min_num_frames_between_kfs)
    {
      VLOG(40) << "KF Select: NO NEW KEYFRAME We just had a KF";
      return false;
    }

    if(n_tracked_fts < options_.kfselect_numkfs_lower_thresh)
    {
      VLOG(40) << "KF Select: NEW KEYFRAME Below lower bound";
      return true;
    }

    

How id works:
There are two kinds of ID, FrameBundleID and Frame ID. A framebundle contains the frames at the same time.
Frames are coverted from cv_mat and inserted to framebundle in FrameHandlerBase::addImageBundle().
The Frame class has a counter that increments everytime a frame is created and it is used for id.
So for mono, FrameBundleID = FrameID, but not the case for stereo and above.

publisRresults():
  visualizer_ handles the publishing

updateSeeds():
Only update the uncertainty/depth of the seeds, does not upgrade to landmarks

upgradeSeedsToFeatures():
upgrade seeds to landmarks, if not enough features (< min features) will upgrade unconverged seeds 


### Sparse Image alignment pipeline
the sparse image alignment module has the following inheritance:
minileastsquare_solver -> sparse image align base -> sparse image align
In the frame_handler_base, it passes the frame bundles to the run() function, which optimizes the frames
see line 73 where the transformations are loaded in to states for optimization
see line 103 goes through each frame and saves the new optimized frame
in the end of the function, it returns the number of tracked feature points.

Sparse Image alignment solves the transformation from last frame to current frame.
It incrementally optimize, starting with identity transformation, the objective to reduce the photometric error of the 
known 3D points that are visible in the last frame. The residual is compute between the patches of pixels of the 3D points
projection in both frames.

### Pose Refinement


### Questions while reading SVO code

If mono can't retreive scale and stereo we can, is there a general framework that works for both?
using principal method such as thresholding self pose uncertainty to do keyframe selection

Is there Direct RGB VO? I would imagine illumination change is not that big of a problem for RGB than for grey scale

Who calls the processFrameBundle fnction?
it happens in the subscribeImage() callback

in svo/common/frame.h:
frame is an object that stores the image, features and pose. so is frame a keyframe? 

In mono, how does process first frame already have two frames?

it seems like both pose and structure are optimizied in the code? I recall the paper said only the pose was optimized in tracking?

Why is select keyframe after all the tracking steps? is this not two parallel thread?

How is new keyframe selected? check frame_handler_base/needNewKf function:

For downfacing camera:
create a new kf if the current depth change is greater than some threshold. why do we do this? if we move slowly upward, then we never create a new frame?

For forward facing camera:


Why does downfacing require special treatment?

Look into why keyframe selection for mono works, but not for others?

#### frame vs frame bundle
frame bundle wraps frame around, 
for mono there is one frame in the frame bundle
for stereo, it stores the frames from the two cameras at this time instance in it.

####cpp question:
frame_handler_mono.cpp line 54
what is {img}

### glog tutorial

Ref:
http://rpg.ifi.uzh.ch/docs/glog.html

### setup catkin config
https://www.personalrobotics.ri.cmu.edu/software/development-environment

### Log to file in launch file
through glog we can do this
<node pkg="svo_ros" type="svo_node" name="svo" clear_params="true" output="screen" args='--v=1 --log_dir=/home/jkuo/' >

### Camera Driver
follow this link:
https://github.com/uzh-rpg/rpg_bluefox
#### launch and change param
for fisheye:
roslaunch svo_ros live_nodelet_fisheye.launch serial_nr:=25001527
for pinhole:
roslaunch svo_ros live_nodelet.launch serial_nr:=25001527

add these two lines under the camera driver section of the launch file if want autoexposure.
May need it if want to work with low light area like the floor:
<param name="cam_ctrl_mode" value="AutoExposureGain" />
<param name="ae_method" value="Intensity" />

In svo_ros/param/pinhole, change to true if want enable autoexposure:
img_align_est_illumination_gain: false
img_align_est_illumination_offset: false

### RPG datasets
https://github.com/uzh-rpg/rpg_datasets

#### vfr dataset
download the dataset from here:http://rpg.ifi.uzh.ch/fov.html
unzip and put all under in this folder /rpg_datasets/rpg_vfr_pinhole/rpg_vfr
that directory contains the calibration file
run this to create the bag:
rosrun rpg_datasets images_to_bag.py /home/jkuo/rpg_work/src/rpg_datasets/rpg_vfr_pinhole/rpg_vfr/

since the ground truth does not have timestamp and for trajectory eval we need ground truth to have timestamp
images_and_poses_to_bag.py in script is created to attatch the image time to ground truth because it is synthetic
then you can use bag_to_pose.py from traj eval to extract the ground_truth.txt

#### euroc dataset
rosbag play MH_01_easy.bag -s 50
rosbag play MH_02_easy.bag -s 45
rosbag play MH_03_medium.bag -s 45
rosbag play MH_04_difficult.bag -s 25
rosbag play V1_01_easy.bag
rosbag play V1_02_medium.bag -s 13
rosbag play V2_01_easy.bag
rosbag play V2_02_medium.bag -s 13

V2_02 entropy 0.95 fail

create bag with images and imu:
rosrun rpg_datasets stereo_images_and_imus_from_euroc_to_bag.py in the data folder

### RPG Trajectory Evaluation
https://github.com/uzh-rpg/rpg_trajectory_evaluation
#### vfr dataset
remove id from ground truth using:
rosrun 

roslaunch svo_ros run_from_bag_vfr_pinhole.launch
rosbag play ~/rpg_work/src/rpg_datasets/rpg_vfr_pinhole/rpg_vfr/out.bag

record a bag with the estimated pose:
rosbag record /svo/pose_imu /svo/image/0

convert the bag into required format:
rosrun rpg_trajectory_evaluation bag_to_pose.py /home/jkuo/side_project/svo_results/rpg_vfr/svo_results.bag /svo/pose_imu --msg_type PoseWithCovarianceStamped

run evaluation:
rosrun rpg_trajectory_evaluation analyze_trajectory_single.py /home/jkuo/side_project/svo_results/rpg_vfr/

#### euroc dataset
strip id from groundtruth.txt:
rosrun rpg_trajectory_evaluation strip_gt_id.py groundtruth.txt
roslaunch svo_ros euroc_mono_imu.launch
rosbag play ~/side_project/datasets/euroc_MH_01_easy/out.bag -s 50
rosbag record /svo/pose_imu /svo/image/0
rosrun rpg_trajectory_evaluation bag_to_pose.py ./svo_results.bag /svo/pose_imu --msg_type PoseWithCovarianceStamped
mv stamped_poses.txt stamped_traj_estimate.txt
rosrun rpg_trajectory_evaluation analyze_trajectory_single.py /home/jkuo/side_project/laptop/svo_forward_default/euroc_MH_01_easy/

#### compare multiple trajectory
rosrun rpg_trajectory_evaluation analyze_trajectories.py --results_dir /home/jkuo/side_project/results/ --output_dir /home/jkuo/side_project/results/evaluation  --platform laptop --odometry_error --overall_odometry_error --plot_trajectories --rmse_table

### entropy evaluation script
python ~/rpg_work/src/rpg_svo_pro/plot_entropy.py

### online entropy evaluation
rqt
subscribe to /svo/pose_refinement_entropy

### SVO building logging trace
need to set TRUE to the SvoSetup.cmake in svo_cmake
trace dir: /home/jkuo/rpg_work/src/rpg_svo_pro/svo/trace

### filters in entropy monitor
#### incremental averaging by each keyframe
Ref:
https://dsp.stackexchange.com/questions/811/determining-the-mean-and-standard-deviation-in-real-time

### Working with Simulation
#### catkin build
catkin build gazebo_simulation_adr on branch origin/multicam_robust
catkin build svo_ros branch origin/multicam_robust
catkin build trajectory_generation_example
catkin build rpg_gazebo_models
catkin build rqt_svo

source devel/setup.bash

#### Edit the world
roslaunch gazebo_simulation_adr gazebo_empty_world.launch
use save as
#### Recrod dataset
roslaunch gazebo_simulation_adr multicam_gazebo_simulation.launch

roslaunch svo_ros

roslaunch trajectory_generation_example generate.launch

rosbag record /hummingbird/ground_truth/imu /hummingbird/ground_truth/pose_with_covariance /hummingbird/imu /hummingbird/rgb_camera/camera_1/camera_info /hummingbird/rgb_camera/camera_1/image_raw /hummingbird/rgb_camera/camera_2/camera_info /hummingbird/rgb_camera/camera_2/image_raw /hummingbird/rgb_camera/camera_3/camera_info /hummingbird/rgb_camera/camera_3/image_raw /hummingbird/rgb_camera/camera_4/camera_info /hummingbird/rgb_camera/camera_4/image_raw /hummingbird/rgb_camera/camera_5/camera_info /hummingbird/rgb_camera/camera_5/image_raw

### Work with Ceres backend benchmarking
use this branch: https://github.com/uzh-rpg/rpg_svo_pro/tree/ceres-backend-dev/svo_ceres_benchmarking
git clone https://github.com/ethz-asl/ceres_catkin
git clone https://github.com/ethz-asl/suitesparse

Prepare a experiment configuartion file

#### change data directory
https://github.com/uzh-rpg/rpg_svo_pro/blob/1930244d849cf2bf72d46c900b958e24b10c541d/svo_ceres_benchmarking/scripts/benchmark.py#L163

#### run experiment
rosrun svo_ceres_benchmarking benchmark.py euroc_multicam_entropy0point96_nolc.yaml

#### run evaluation with multi run
use this repo until it is merged: 
https://github.com/zhangzichao/rpg_trajectory_evaluation

need to setup the analyze_trajectories_config

rosrun rpg_trajectory_evaluation analyze_trajectories.py euroc_ceres_backend.yaml --output_dir=/home/jkuo/rpg_work/src/rpg_svo_pro/svo_ceres_benchmarking/results/evaluation --results_dir=/home/jkuo/rpg_work/src/rpg_svo_pro/svo_ceres_benchmarking/results --platform laptop --odometry_error_per_dataset --plot_trajectories --rmse_table --rmse_boxplot --mul_trials=5

### Calibrate with Kalibr with Images
Create the clibration bag from images using:
python kalibr_bagcreater --folder /home/jkuo/2017-11-16_Geometric_Calibration/GeometricCalibration_Front --output-bag 2017-11-16_GeometricCalibration_Front.bag

Note: the bag creater requirse the name of the images to be in nanoseconds, but autovisoin calibration sequence does not have time as name.
So we create the timestamp by incrementing seconds in the image sequence order.

### Working with multicam dataset from CVG
Need to download this: 
https://github.com/USCiLab/cereal
and put it in the include folder or find it somehow in your cmake file
ADD_LIBRARY(cereal INTERFACE)

TARGET_INCLUDE_DIRECTORIES(
cereal
INTERFACE include
)
