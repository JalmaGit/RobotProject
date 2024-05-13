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

from . import Kinematics

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from angle_msg.msg import Pitchroll
import math

class KinematicsCom(Node):

    #Subscribe to roll and pitch topic
    #Publish Motor Angle joints

    def __init__(self):
        self.kinematics = Kinematics.Kinematics()
        super().__init__('ThreeDofCommunicator')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(Pitchroll, 'pitch_roll_message',
            self.listener_callback, 10)
        self.subscription
       
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0
        self.last_publish_time = self.get_clock().now()

        self.pitchroll_msg = Pitchroll()

    def listener_callback(self, msg):
        self.pitchroll_msg.roll = msg.roll
        self.pitchroll_msg.pitch = msg.pitch
        self.get_logger().info('I heard: "%d"' % msg.roll) # CHANGE

    def timer_callback(self):
        msg = JointTrajectory()
    
        current_time = self.get_clock().now()
        msg.header.stamp = current_time.to_msg()

        msg.joint_names = ['ard_motorA', 'ard_motorB', 'ard_motorC'] 
        
        points = JointTrajectoryPoint()
        #roll = round(1*math.sin(self.i),1)
        #pitch = round(1*math.cos(self.i),1)

        newPoints = self.kinematics.inverse_kinematics(self.pitchroll_msg.roll, self.pitchroll_msg.pitch, 0)
        print(newPoints)

        points.positions = newPoints
        
        duration_since_last_publish = current_time - self.last_publish_time
        points.time_from_start = duration_since_last_publish.to_msg()

        msg.points.append(points)

        self.publisher_.publish(msg)
        self.last_publish_time = current_time
        self.get_logger().info('Publishing: "%s"' % points.positions)
        self.i += 0.5


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
