# RRT star with multiple remote goals and Pure Pursuit with PID controllers.

This repository contains package with code implementation of RRT star-based path planning algorithm for finding out a suitable path between the cones.

### Lidar based
This is completly based on lidar data (velodyne 16) and therefore there is a need to install pcl in your system for point cloud processong and conversion of point cloud to cones cluster and thereby finding out their coordinates.

### Notes
We just use LIDAR data to mark each cones with  a coordinate and covert each coordinate into list and use it for path planning.

#### Parameters to tune (main)
- `odom_topic` = `/odometry` - The topic to get the odometry information from
- planDistance = 12 m - Maximum length of tree branch
- expandDistance = 1 m - Length of tree node/step
- expandAngle = 20 deg - constraining angle for next tree nodes
- coneObstacleSize = 0.5 m - size of obstacles derived from cone position
- coneTargetsDistRatio = 0.5 - ratio to frontConesDist for deriving remote cone goals

### Simulation Video for Path Planning


https://user-images.githubusercontent.com/67441175/125800769-db53241a-ecb6-4888-b08e-d9a2db8d7ade.mp4


### Commands (For generating suitable path)
- roslaunch eufs_gazebo small_track.launch 
- roslaunch robot_control robot_control.launch 
- rosrun pointcloud_process final_script.py
- roslaunch path_planning initiation.launch

# Control
Pure pursuit and PID controllers are used in this autonomous stack.
Pure pusuit is a lateral controller and is suitable for our problem statement. It suitable for high speed than other controllers like stanley which gives another reason for it to be chosen. PID is a longitudnal controller which is tries to maintain the target velocity given to it. 
Thus we need another script for velocity profile generation. Instantaneous velocity on the track is chosen to be a function of yaw rate which is itself a function of longitudnal velocity and steering angle. Further parameter tuning can be done for better results.     

### Simulation Video for Controls



https://user-images.githubusercontent.com/67441175/125802857-9a407f06-1d96-4428-ab55-076ce61c1c51.mp4




### Commands (Fixed waypoints for controls)
- roslaunch eufs_gazebo small_track.launch
- rosrun path_planning control_initiate.py
- rosrun path_planning control_listpath.py
- rosrun path_planning pid_launcher.py
   
