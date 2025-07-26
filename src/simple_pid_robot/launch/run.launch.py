from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch
import yaml

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_pid_robot')
    ld = LaunchDescription()
    params = os.path.join(pkg_dir, 'config', 'config.yaml')
    
    with open(params, 'r') as f:
        config = yaml.safe_load(f)    
        print(config)
    field = Node(
        package='simple_pid_robot',
        namespace='',
        executable='field',
        name='field_node',
        arguments=[params, {"image_topic": config["image_topic"]}]
    )
    robot = Node(
        package='simple_pid_robot',
        namespace='',
        executable='robot',
        name='robot_node',
        arguments=[params, {"image_topic": config["image_topic"]}]
    )

    rqt_image_view = Node(
        package='rqt_image_view',
        namespace='',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=[config["image_topic"]]
    )
    ld.add_action(field)
    ld.add_action(robot)
    ld.add_action(rqt_image_view)
    return ld