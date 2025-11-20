import os
import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer 
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown
#写一个launch.py
def generate_launch_description():
    package_share_directory = get_package_share_directory('tf_tree')
    config = os.path.join(package_share_directory, 'config', 'config.yaml')
    
    tetris = ComposableNodeContainer(
        name='tetris',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        prefix='gdb -ex run --args',
        emulate_tty=True,
        on_exit=Shutdown(),
        # arguments = ["--ros-args", "--log-level", "debug"],
        composable_node_descriptions=[
            ComposableNode(
                package = 'tetris',
                plugin = 'TetrisNode',
                name = 'tetris_node',
                extra_arguments = [{ "use_intra_process_comms": True }],
                parameters = [config]
            )
        ]
    )
    tf_tree= ComposableNodeContainer(
        name='tf_tree',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        # arguments = ["--ros-args", "--log-level", "WARN"],
        composable_node_descriptions=[
            ComposableNode(
                package = 'tf_tree',
                plugin = 'TF2Node',
                name = 'tf2_node',
                extra_arguments = [{ "use_intra_process_comms": True }],
                parameters = [config]
            )
        ]
    )

    xarm_robot = ComposableNodeContainer(
        name='xarm_robot',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # prefix='gdb -ex run --args',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        # arguments = ["--ros-args", "--log-level", "WARN"],
        composable_node_descriptions=[
            ComposableNode(
                package = 'xarm_robot',
                plugin = 'XarmNode',
                name = 'xarm_robot_node',
                extra_arguments = [{ "use_intra_process_comms": True }],
                parameters = [config]
            )
        ]
    )

    camera = ComposableNodeContainer(
        name='camera',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # prefix='gdb -ex run --args',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        # arguments = ["--ros-args", "--log-level", "WARN"],
        composable_node_descriptions=[
            ComposableNode(
                package = 'camera',
                plugin = 'CameraNode',
                name = 'camera_node',
                extra_arguments = [{ "use_intra_process_comms": True }],
                parameters = [config]
            )
        ]
    )



    # return LaunchDescription([ xarm_robot, camera, tf_tree, tetris  ]);
    return LaunchDescription([ xarm_robot  ]);