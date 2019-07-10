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

### running deepsort
python deep_sort_app.py --sequence_dir=./MOT16/MOT16-06 --detection_file=./detections/MOT16-06.npy --min_confidence=0.3 --nn_budget=100 --display=True