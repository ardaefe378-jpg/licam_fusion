# **licam\_fusion**



A ROS2 package of 3D LiDAR-Camera early-fusion and depth estimation using OpenCV and Eigen.



### **IO**



* **input**

/camera/color/camera\_info  (sensor\_msgs/CameraInfo)

/perception/stereo\_detections  (vision\_msgs/Detection2DArray)

/points\_downsampled (sensor\_msgs/PointCloud2)

tf (tf2\_msgs/TFMessage)(from lidar\_frame\_id to camera header frame)



* **output**

/rosout (rcl\_interfaces/Log)(Object ID and median depth logged to standard output)



### **Config**



|NAME|TYPE|DEFAULT VALUE|DESCRIPTION|
|-|-|-|-|
|camera\_info\_topic|string|"/camera/color/camera\_info"|intrinsic calibration topic|
|bbox\_topic|string|"/perception/stereo\_detections"|2D bounding box topic|
|lidar\_topic|string|"/points\_downsampled"|3D point cloud topic|
|lidar\_frame\_id|string|"cloud"|TF root frame of LiDAR|
|sync\_queue\_size|int|15|ApproximateTime message queue tolerance|



### **demo**



Before running, configure your specific sensor topics and TF frame IDs in the fusion\_params.yaml file located in the config directory.
Ensure you have tf2 publishing the transform between the camera and LiDAR frames.



colcon build --packages-select licam\_fusion --symlink-install

source install/setup.bash

ros2 launch licam\_fusion fusion.launch.py

