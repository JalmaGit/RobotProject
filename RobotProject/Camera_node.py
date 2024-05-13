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

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image


class Camera(Node):

    def __init__(self):
        super().__init__('Camera_Feed_SubPub')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 0)
        timer_period = 1.0/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cvBridge = CvBridge()
        self.camera = cv2.VideoCapture(2)
    
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.i = 0

    def timer_callback(self):

        ret, frame = self.camera.read()

        if ret:
            try:
                msg = self.cvBridge.cv2_to_imgmsg(frame, "bgr8")
                #msg = self.cvBridge.cv2_to_compressed_imgmsg(frame, "bgr8")

                self.publisher_.publish(msg)
                #self.get_logger().info('Publishing: "%s"' % self.i)
            except Exception as e:
                self.get_logger().warn(f"Error converting frame to Image message: {str(e)}")
        else:
            self.get_logger().warn("Failed to capture frame from camera")
        
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Camera()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
