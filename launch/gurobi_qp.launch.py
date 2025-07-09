from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gurobi_ros2_testing')
    param_file = os.path.join(pkg_share, 'config', 'param.yaml')

    return LaunchDescription([
        Node(
            package='gurobi_ros2_testing',
            executable='qp_solver',
            name='qp_solver', 
            parameters=[param_file],
            output='screen',
        )
    ])
