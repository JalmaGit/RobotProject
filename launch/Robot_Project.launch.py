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

    Camera_node = Node(
        package='RobotProject',
        executable='Camera'
    )

    Ball_Detection_node = Node(
        package='RobotProject',
        executable='ball_detection_node'
    )

    ld.add_action(Controller_node)
    ld.add_action(Kinematics_node)
    ld.add_action(Ball_Detection_node)
    ld.add_action(Camera_node)

    return ld