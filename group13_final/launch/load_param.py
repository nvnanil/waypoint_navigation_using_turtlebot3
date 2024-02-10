import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Main function for the launch file
    """
    # find the parameter file
    parameter_file = os.path.join(
        get_package_share_directory('group13_final'),
        'config',
        'waypoint_params.yaml'
    )
    aruco_listen = Node(
    package="group13_final",
    executable="aruco_listen",
    parameters=[parameter_file]
    )
      
    ld = LaunchDescription()
    ld.add_action(aruco_listen)
    return ld