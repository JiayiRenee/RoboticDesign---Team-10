import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'enable_color': True,  # ✅ 启用彩色图像
                'enable_depth': True,  # ✅ 启用深度图像
                'enable_infra': False,  # ❌ 关闭红外图像
                'enable_pointcloud': True,  # ✅ 启用点云
                'align_depth': True,  # ✅ 深度图像对齐到彩色图像
                'color_width': 640,  # ✅ 颜色图像宽度
                'color_height': 480,  # ✅ 颜色图像高度
                'color_fps': 30,  # ✅ 颜色图像帧率
                'depth_width': 640,  # ✅ 深度图像宽度
                'depth_height': 480,  # ✅ 深度图像高度
                'depth_fps': 30,  # ✅ 深度图像帧率
                'enable_sync': True,  # ✅ 颜色和深度同步
                'enable_auto_exposure': True,  # ✅ 自动曝光
                'initial_reset': False  # ❌ 启动时不重置相机
            }]
        )
    ])

