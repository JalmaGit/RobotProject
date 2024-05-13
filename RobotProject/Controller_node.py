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
import math
import time

class Controller_node(Node):

    #Subscribe to roll and pitch topic
    #Publish Motor Angle joints

    def __init__(self):
        self.PIDx = Controller.PID(0.04, 0.0, 0.0)
        self.PIDy = Controller.PID(0.04, 0.0, 0.0)
        
        super().__init__('Controller_node')
        self.publisher_ = self.create_publisher(Pitchroll, 'pitch_roll_message', 10)
        self.subscription = self.create_subscription(Point, 'ball_position',
            self.listener_callback, 10)
        self.subscription
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.last_time = time.time()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.x) # CHANGE

    def timer_callback(self):
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        msg = Pitchroll()

        roll = 0.5*math.cos(self.i)
        pitch = 0.5*math.sin(self.i)

        #roll = self.PIDx.control(0, xPos, delta_time)
        #pitch = self.PIDy.control(0, yPos, delta_time)

        msg.roll = roll
        msg.pitch = pitch

        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.roll)
        self.i += 1


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