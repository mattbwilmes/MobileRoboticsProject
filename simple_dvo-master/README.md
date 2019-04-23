# Simple DVO Method

Used the code written by muskie82 here: https://github.com/muskie82/simple_dvo

Related paper: https://ieeexplore.ieee.org/document/6631104

Tested with: Ubuntu 16.04 with ROS kinect and MATLAB 2018b

## Dependencies

- Ubuntu 16.04
- ROS kinetic
- OpenCV
- PointCloudLibrary

## Procedure

### Rviz instructions from muskie82
The first part of these instructions are based on the README.md by muskie82 here: https://github.com/muskie82/simple_dvo

1. Download a rosbag from the TUM RGB_D dataset: https://vision.in.tum.de/data/datasets/rgbd-dataset/download
  - For this example, we will use `rosbag_dataset_freiburg1_desk.bag`
2. `roscore`
3. `rosrun simple_dvo main /camera/rgb/image_color /camera/depth/image /camera/rgb/camera_info`
4. `rosrun rviz rviz`
5. In rviz, "Fixed Frame" tab, change "world" to "cam_origin"
6. "Add" -> "By topic" and add "PointCloud2"
7. "Add" -> "By display type" and add "TF"

### Generating .txt file to be read by MATLAB code
This next part was created by us in order to generate a .txt file necessary for our analysis pipeline

1. Stop the rosbag by hitting Ctrl+C
2. Start recording a new rosbag containing only the /tf rostopic:
- `rosbag record -O truncated /tf`
3. Start playing the datasest rosbag
- `rosbag play rgbd_dataset_freiburg1_desk.bag`
4. Stop recording truncated.bag once rgbd_dataset_freiburg1_desk.bag finishes by hitting Ctrl+C
5. Save output in a condensed format in a temp.txt file
- `rostopic echo -b truncated2.bag -p /tf > temp.txt`
6. Extract releveant information from temp.txt and store in .txt file
-  `cat temp.txt | grep -B 1 -A 1 ",camera," > freiburg1_desk_tforms_newest.txt`

### Generating final .mat file for our Analysis Pipeline
This last part converts the .txt file into a .mat file readable by our analysis pipeline

1. Modify read_dvo_tf_file.m so that `tf_file` equals the location of your newly generated .txt file
2. Run read_dvo_tf_file.m in MATLAB
- This will save the variable transformation as a .mat file called freiburg1_desk_tforms_newest.mat
3. Run camera_to_world.m
- `camera_to_world('s_dvo_assG_RGB_Desk.txt','freiburg1_desk_tforms_newest.mat','freiburg1_desk_frame_tforms.mat')`
- This will output a .mat file freiburg1_desk_frame_tforms.mat that can be read by our analysis pipeline

## Folder Struture

simple_dvo-master

-->build/ - used for Cmake

-->devel/ - used for Cmake

-->freiburg1_desk/ - relevant files for fr1/desk dataset

-->freiburg1_xyy/ - relevenat files for fr1/xyz dataset

-->install/ - used for Cmake

-->src/ - used for Cmake and contains code from muskie82

-->assoc.txt - used for analysis

-->camera_to_world.m - used to generate final .mat file of frame-to-frame transformations

-->read_dvo_tf_file.m - used to convert .txt files from ROS to .mat files of frame-cam_origin transformations
