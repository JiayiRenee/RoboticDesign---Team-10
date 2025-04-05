
## 🔧 Requirements

- ROS 2 Humble
- Python 3.10+
- Intel RealSense camera (e.g., D435i)
- Dependencies:
  - OpenCV
  - numpy
  - pyrealsense2
  - sensor_msgs, std_msgs, geometry_msgs, cv_bridge

## 🚀 Running the Node

You can run the detection node directly (without launch file):
colcon build
source
```bash
ros2 run object_detection object_detection

Topic Name	Message Type	Description
/color_detection	std_msgs/String	Text-based log of detected objects
/color_image	sensor_msgs/Image	Annotated image stream
/block_pose	geometry_msgs/Pose	3D position of detected blocks
/bin_pose	geometry_msgs/Pose	3D position of detected bins

Integration with Robotic Arm
Your robotic arm node can subscribe to:

/block_pose: For pick-up targets

/bin_pose: For placing into the correct bin

