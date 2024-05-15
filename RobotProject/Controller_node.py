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

from . import Controller

from geometry_msgs.msg import Point
from angle_msg.msg import Pitchroll
import numpy as np
import time

class Controller_node(Node):

    #Subscribe to roll and pitch topic
    #Publish Motor Angle joints

    def __init__(self):
        kp = 0.0080 #0.004 #0.0008#0.007 #0.4 Max
        ki = 0.00020 #0.00050 #0.001#0.001
        kd = 0.0045 #0.00065  #0.00085 #0.0075#0.008

        self.PIDx = Controller.PID(kp, ki, kd)
        self.PIDy = Controller.PID(kp, ki, kd)
        
        super().__init__('Controller_node')
        self.publisher_ = self.create_publisher(Pitchroll, 'pitch_roll_message', 10)
        self.subscription = self.create_subscription(Point, 'ball_position',
            self.listener_callback, 10)
        self.subscription
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_time = time.time()
        self.point = Point()
        self.flagg = False

    def listener_callback(self, msg):
        self.flagg = True
        self.point = msg

    def timer_callback(self):
        if self.flagg:    
            self.flagg = False

            current_time = time.time()
            delta_time = current_time - self.last_time
            self.last_time = current_time

            msg = Pitchroll()

            centerpoint = [340, 244]

            #roll = (340 - self.point.x)/100
            #pitch = (244 - self.point.y)/100

            roll = self.PIDx.control(centerpoint[0], self.point.x, delta_time)
            pitch = self.PIDy.control(centerpoint[1], self.point.y, delta_time)

            msg.roll = roll
            msg.pitch = pitch

            self.publisher_.publish(msg)
            self.get_logger().info('Publishing roll: "%s"' % msg.roll)
            self.get_logger().info('Reading I: "%s"' % self.PIDx.I)


# The names of the active joints in each trajec
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Controller_node()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()