from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    Controller_node = Node(
        package='RobotProject',
        executable='Controller'
    )

    Kinematics_node = Node(
        package='RobotProject',
        executable='Kinematics'
    )

    ld.add_action(Controller_node)
    ld.add_action(Kinematics_node)

    return ld