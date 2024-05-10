# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from . import CameraVision
from . import Controller
from . import Kinematics

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image
import math

class KinematicsCom(Node):

    #Subscribe to roll and pitch topic
    #Publish Motor Angle joints

    def __init__(self):
        self.kinematics = Kinematics.Kinematics()
        super().__init__('ThreeDofCommunicator')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC'] 

        points = JointTrajectoryPoint()
        roll = round(1*math.sin(self.i),1)
        pitch = round(1*math.cos(self.i),1)

        print(roll)

        #points.positions = [point, point, point]

        newPoints = self.kinematics.inverse_kinematics(roll, pitch, 0)
        print(newPoints)

        points.positions = newPoints
        
        duration_msg = Duration()
        duration_msg.sec = 0  # You can adjust these values based on your requirements
        duration_msg.nanosec = 90000000  # For example, 0.1 seconds
        points.time_from_start = duration_msg

        msg.points.append(points)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % points.positions)
        self.i += 1


# The names of the active joints in each trajec
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = KinematicsCom()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
