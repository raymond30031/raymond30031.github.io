### Installation
follow the instructions in the rpg_svo_pro repo readme

### run test
roslaunch svo_ros run_from_bag.launch cam_name:=svo_test_pinhole

rosbag play /media/jkuo/Data/side_project/airground_rig_s3_2013-03-18_21-38-48.bag

set the namespace to svo, then it would show # of tracked feature

### pipeline
svo_node -> subscribeImage() -> new thread for monoLoop -> set up subscriber for monoCallBack():
process image bundle and publish results
note svo_ is a framehandler 
-> processImageBundle -> addImageBundle -> addFrameBundle -> processFrameBundle

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

####cpp question:
frame_handler_mono.cpp line 54
what is {img}

### Test code

source ~/work/Mask_RCNN/venv2.7/bin/activate
python
import sys
sys.path.append("/home/jkuo/work/Mask_RCNN/") // need this to import mrcnn.model
sys.path.append("/home/jkuo/work/Mask_RCNN/samples/maplab_interface")
import maplab_interface as interface
model = interface.create_model()