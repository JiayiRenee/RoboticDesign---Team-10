from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):



    robot_model_launch_arg = LaunchConfiguration('robot_model')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    robot_name_launch_arg = LaunchConfiguration('robot_name')


    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])


    robot_description_semantic = construct_interbotix_xsarm_semantic_robot_description_command(
        robot_model=robot_model_launch_arg.perform(context),
        config_path=config_path,
    )
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )



    moveit_interface_node = Node(
        package='px150_move_group_commander', #interbotix_moveit_interface
        executable='px150_move_group_commander_node',#moveit_interface
        # namespace=robot_name_launch_arg,
        # condition=LaunchConfigurationEquals(
        #     'moveit_interface_type',
        #     expected_value='cpp'
        # ),
        parameters=[{
            'robot_description_semantic': robot_description_semantic,
            'use_sim_time': use_sim_time_param,
        }],
        remappings=(
            ('/joint_states', f'/{robot_name_launch_arg.perform(context)}/joint_states'),
            ('/robot_description', f'/{robot_name_launch_arg.perform(context)}/robot_description'),
        )
    )
    return[
        moveit_interface_node
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_interbotix_xsarm_models(),
            description="model type of the Interbotix Arm such as 'wx200' or 'rx150'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description=(
                'the file path to the custom semantic description file that you would '
                "like to include in the Interbotix robot's semantic description."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit_interface'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='world',
            description=(
                'defines the fixed frame parameter in RViz. Note that if '
                '`use_world_frame` is `false`, this parameter should be changed to a frame'
                ' that exists.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit_interface'),
                'rviz',
                'xsarm_moveit_interface.rviz'
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_common_sim'),
                'worlds',
                'interbotix.world',
            ]),
            description="the file path to the Gazebo 'world' file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_interface_type',
            default_value='cpp',
            choices=(
                'cpp',
                # 'python',
            ),
            description=(
                "if 'cpp', launches the custom moveit_interface C++ API node; if 'python', launch "
                'the Python Interface tutorial node; only the cpp option is currently supported.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_interface_gui',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'launch a custom GUI to interface with the moveit_interface node so that the user '
                "can command specific end-effector poses (defined by 'ee_gripper_link')."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                "published over the ROS topic /clock; this value is automatically set to 'true' if"
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
