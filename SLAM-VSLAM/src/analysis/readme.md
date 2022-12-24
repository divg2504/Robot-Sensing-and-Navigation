#  RTAB-MAP
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
- Launch the rtabmap node using:
- roslaunch rtabmap_ros rtabmap.launch \
   stereo:="true" \
   left_image_topic:=/kitti/camera_color_left/image_raw \
   right_image_topic:=/kitti/camera_color_right/image_raw \
   left_camera_info_topic:=/kitti/camera_color_left/camera_info \
   right_camera_info_topic:=/kitti/camera_color_right/camera_info \
   rtabmap_args:="--delete_db_on_start \
      --RGBD/CreateOccupancyGrid false \
      --Rtabmap/ImageBufferSize 0 \
      --Odom/ImageBufferSize 0 \
      --Rtabmap/CreateIntermediateNodes true" \
   approx_sync:=false \
   use_sim_time:=true \
   frame_id:=base_link \
   queue_size:=100

- IN RTABMAP GUI, SET THE PARAMETERS USING FOLLOWING CONFIGURATION FILE:
In window-> Preferences-> General Settings-> Load Config-> kitti_config.ini 
- PLAY ROSBAG FILES USING FOLLOWING COMMAND
rosbag play --clock -r 0.2  kitti_2011_10_03_drive_0027_synced.bag

#  ORB-SLAM3

- REQUIRED DEPENDENCIES (Ubuntu 20.04)

1. sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
2. sudo apt update
3. sudo apt-get install build-essential
4. sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
5. sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
6. sudo apt-get install libglew-dev libboost-all-dev libssl-dev
7. sudo apt install libeigen3-dev
8. sudo apt-get install libcanberra-gtk-module

- REQUIRED OpenCV VERSION - 3.2.0

- CLONE THE ORB-SLAM3 REPOSITORY - git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git inside the '/Dev' folder

- DOWNLOAD THE KITTI DATASET(ODOMETRY) IN ~/Datasets FOLDER

- RECTIFYING THE CODE FILES

-> gedit ./modules/videoio/src/cap_ffmpeg_impl.hpp
Write the following lines at the top 
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
#define AVFMT_RAWPICTURE 0x0020

-> gedit ./include/LoopClosing.h
Replace Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose; with Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;

-> gedit ./src/System.cc 
Replace Map* pBiggerMap; with Map* pBiggerMap = nullptr;

-> Run the following commands in the terminal before executing the algorithm
(1) sed -i’s/++11/++14/g’ CMakeLists.txt
(2) sudo ldconfig

- TO RUN THE ALGORITHM
Run the following commands sequentially:
(1) cd ~/Dev/ORB_SLAM3 
(2) ./Examples_old/Stereo/stereo_kitti_old ./Vocabulary/ORBvoc.txt ./Examples_old/Stereo/KITTI00-02.yaml ~/Datasets/Kitti/data_odometry_gray/dataset/sequences/00

# GRAPH SLAM
Github repo link:
https://github.com/koide3/hdl_graph_slam.git

Launches the nodes:
roslaunch hdl_graph_slam hdl_graph_slam_kitti_final.launch

For rviz:
rviz -d hdl_graph_slam_kitti.rviz

# LEGO LOAM

1. Download the src folder into an empty workspace folder. Navigate to your workspace folder in terminal and then run **catkin_make -j1** in the terminal.

2. Change the **Result_dir** to a local directory in the run.launch file in the kitti-lego-loam package. Run the following command: **roslaunch lego_loam run.launch** to run LeGO LOAM. This should result in an RViz window being opened.

3. Run a bag file using the following command: **rosbag play (Bag_file_Name) --clock**

4. The resultant poses will be saved in the re_00.txt in the Result_dir.

Note: The following repository was used to compare the estimated trajectories and the ground truth posess: https://github.com/MichaelGrupp/evo
Note: The following repository was used as a reference for developing code: https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/


# ROAD SIGN DETECTION
The following Kaggle Notebook: https://www.kaggle.com/code/arshtangri2/kitti-traffic-sign-detection , was used for writing the Road Sign Detection Code.


# Convert poses from KITTY format to TUM format and then map ROAD SIGNS ON 2D TRAJECTORY
1. python kitti_to_tum.py dataset_path/sequences/00/poses.txt dataset_path/sequences/00/timeseries.txt dataset_path/sequences/00/poses_tum.txt
2. python map_road_signs.py dataset_path/sequences/00/poses_tum.txt dataset_path/sequences/00/sign_dict.pickle
