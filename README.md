# RoboticDesign---Team-10




RealSense Detection Package Instructions for Use (object_detection package)
1）colcon build/source
2）ros2 run object_detection object_detection


Hint: the useful code is mainly in the object_detection.py file, the launch file is a bit problematic at the moment

ROS2 Topics：


Subscribed Topics
1）/camera/color/image_raw	sensor_msgs/msg/Image	
2）/camera/depth/image_rect_raw	sensor_msgs/msg/Image


Published Topics
1）/color_image	sensor_msgs/msg/Image（Color image after target detection (with bounding box)）
2）/color_detection	std_msgs/msg/String（Detected color information）
3）/block_pose	geometry_msgs/msg/Pose（Coordinates of recognized blocks (X, Y, Z)）
4）/bin_pose	geometry_msgs/msg/Pose（Coordinates of the recognized bin (X, Y, Z)）


This can be considered if the robot arm needs to interact:
Listening to the /block_pose topic to receive the coordinates of the target object.
Listening to the /bin_pose topic to receive the position of the trash cans.
subscribing to /color_detection to get color information about the target object

