# True-Color-3D-Reconstruction-Small-Object
This project fuses ToF depth data with RGB images to generate true-color 3D point clouds. Multi-view scans are then aligned using NDT, merged, meshed, and prepared for full-color 3D reconstruction. The project includes two packages: color_depth_fusion, which performs depth-RGB sensor fusion to create true-color point clouds, and lidar_slam_3d, which handles registration of multi-view point clouds for 3D reconstruction.

This guide will help you set up your computer system, compile the code, and run the package. 
 
############ Computer Information ##############
 
Manufacturer: Dell Inc.
	Product Name: OptiPlex 3090
	Version: Not Specified
	Serial Number: JVK3RN3
	UUID: 4c4c4544-0056-4b10-8033-cac04f524e33
	Wake-up Type: Power Switch
	SKU Number: 0B8A
    Family: OptiPlex
	
	
############ Operating System ##################
 
Description:	Ubuntu 20.04.6 LTS
Release:	20.04
Codename:	focal
 
 
############ Build System Generator ############ 
 
cmake version 3.29.2
 
 
############ Software Platform #################
        
ROS Noetic
 
############ Libaraies Installed ###############
 
1) Point Cloud Library (PCL) 1.10       
2) OpenCV 4.6      
3) pylon 26.03 (Linux x86 ) 
   (pylon Software Suite: https://docs.baslerweb.com/pylon-software-suite)
4) ceres-solver (version 2.0)	



############ IP Configurator ###################
## Test and View
## open a teminal
~$ cd /opt/pylon/bin$ 
 
1) ipconfigurator -- IP configuration  
2) blazeTest      -- Test Basler blaze  
3) blazeViwer     -- Viewe 3D point cloud  
 
############ Calibration ########################
  
Step 1) Open a teminal
  1) ~$ cd /opt/pylon/share/pylon/Samples/blaze/cpp/MultiCam/ColorAndDepth/Calibration/build
  2) ~$ make
  3) ~$ ./ColorAndDepthCalibration
 
The calibration file "calibration_24324250_24459894.xml" file is generated in folder: 
/opt/pylon/share/blaze/Samples/blaze/cpp/MultiCam/ColorAndDepth/CalibrationData
 
Step 2) Copy calibration_24324250_24459894.xml file to folder:
~/catkin_ws/src/wound_closure_delta_3dcamera_project/blaze_sensor/ColorAndDepthFusionROS/CalibrationData 
   
############ Compile the package ###############

## Download the package to your ROS workspace: catkin_ws
1) ~$ cd catkin_ws
2) ~/catkin_ws$ git clone https://github.com/WilliamZhao423/True-Color-3D-Reconstruction-Small-Object.git
    
## Compile the package  
3) ~/catkin_ws$ catkin_make
 
If there is no error, you have compiled the package successfully.
 
Before running the pakage, you need to modify the translation parameters with your configuration  of Camera and turn table.
1) Open file:~/catkin_ws/src/ColorAndDepthFusionROS/src/fusion_node.cpp
2) Line 157: Eigen::Translation3f init_translation ( 2, -30, -330);  
3) This vector ( 2, -30, -330) represents the 3D position of the turntable center relative to the camera origin. 

Interpreting this in a typical 3D camera coordinate frame:
    x = 2 → slightly to the right of the camera center
    y = -30 → downward (assuming +y is up, which is common)
    z = -330 → in front of the camera (assuming camera looks along negative z, which is typical in many vision systems.
    
Note: modify parameters reagards your configuration parameters
    1) Open file:~/catkin_ws/src/lidar_slam_3d/src/lidar_slam_3d_ros.cpp
    2) Line 25: private_nh.param("speed_stage_rpm", speed_stage_rpm_, 1.65); 
	   1.65rpm -- 500; 3.3rpm -- 1000
    3) Open file: ~/catkin_ws/src/lidar_slam_3d/launch/lidar_slam_3d.launch
    4) Line 14: <param name="speed_stage_rpm"          value="3.3"/>
    5) Change value=" "

############ Run the package in terminal #######

## Run package in terminal  
1) Open a terminal
   ~$ roslaunch basler_blaze_fusion basler_blaze_fusion.launch
 
2) Open a terminal
   ~$ roslaunch lidar_slam_3d lidar_slam_3d.launch
    
## Visulization
3) Open a terminal
   ~$ rviz
    
   Note: In rviz interafce:    
   Fixed Frame: camera_optical_frame
   Topic: /filtered_points
   
or
 
############ Run the package in RQT ###########
## roscore 
   ~$ roscore 
    
## Start rqt
  ~$ rqt
   
   Note: Open rviz 
   In RViz: File -> Open ConFig -> ~/catkin_ws/src/lidar_slam_3d/launch/scanner_rivz.rviz
        or: Recent ConFigs -> ~/catkin_ws/src/lidar_slam_3d/launch/scanner_rivz.rviz
         
## In ROS launch GUI
## Start blaze-Balser cameras
   basler_blaze_fusion -> basler_blaze_fusion.launch
    
## In ROS launch GUI2
## Point cloud registration
   lidar_slam_3d -> lidar_slam_3d.launch
    
    
   
         
