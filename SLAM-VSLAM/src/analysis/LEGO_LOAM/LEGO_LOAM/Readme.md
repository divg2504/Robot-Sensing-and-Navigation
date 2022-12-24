## LeGO-LOAM

1. Download the src folder into an empty workspace folder. Navigate to your workspace folder in terminal and then run **catkin_make -j1** in the terminal.

2. Change the **Result_dir** to a local directory in the run.launch file in the kitti-lego-loam package. Run the following command: **roslaunch lego_loam run.launch** to run LeGO LOAM. This should result in an RViz window being opened.

3. Run a bag file using the following command: **rosbag play (Bag_file_Name) --clock**

4. The resultant poses will be saved in the re_00.txt in the Result_dir.


Note: The following repository was used to compare the estimated trajectories and the ground truth posess: https://github.com/MichaelGrupp/evo

Note: The following repository was used as a reference for developing code: https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/
