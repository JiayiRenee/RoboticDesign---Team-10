import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # ✅ 1. 启动 RealSense 相机
        launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_pointcloud': False,
                'align_depth': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30
            }]
        ),

        # ✅ 2. 启动目标检测节点
        launch_ros.actions.Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            output='screen'
        ),
    ])

