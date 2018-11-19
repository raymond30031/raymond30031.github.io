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
        processFirstFrame():
          sets first frame as keyframe
          adds first frame as keyframe in map
        processFrame():
          sparseImageAlignment():
          poseOptimization():
    assign the new_frames_ to last_frames (last framebundle that was processed)
  


publish results

### Sparse Image alignment pipeline
the sparse image alignment module has the following inheritance:
minileastsquare_solver -> sparse image align base -> sparse image align
In the frame_handler_base, it passes the frame bundles to the run() function, which optimizes the frames
see line 73 where the transformations are loaded in to states for optimization
see line 103 goes through each frame and saves the new optimized frame
in the end of the function, it returns the number of tracked feature points.

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