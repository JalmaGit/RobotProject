import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class BallDetectionNode(Node):
    """
    A node for detecting a ball in an image using Canny edge detection
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the ball detection node.
        """
        super().__init__('ball_detection')

        #subscribe to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.subscription  #prevent unused variable warning

        #publisher for ball position
        self.ball_position_publisher = self.create_publisher(
            Point,
            'ball_position',
            10)

        #publisher for modified image with detected circles
        self.image_publisher = self.create_publisher(
            Image,
            'image_with_circles',
            10)

        #initialize CVBridge
        self.bridge = CvBridge()

        #load parameters for Canny edge detection
        self.declare_parameter('upper_threshold', 200)
        self.upper_threshold = self.get_parameter('upper_threshold').value

        self.declare_parameter('lower_threshold', 100)
        self.lower_threshold = self.get_parameter('lower_threshold').value

        #define limits for the diameter or radius of the detected object (ball)(might need some more "tuning")
        self.min_radius = 5  #minimum radius of the ball
        self.max_radius = 50  #maximum radius of the ball

    def image_callback(self, msg):
        """
        Callback function for processing image messages
        """
        try:
            #convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        #apply Canny edge detection to detect edges in the image
        try:
            edges = cv2.Canny(cv_image, self.lower_threshold, self.upper_threshold)
        except Exception as e:
            self.get_logger().error('Failed to apply edge detection: %s' % str(e))
            return

        #find circles using Hough Circle Transform
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20,
                           param1=30, param2=15, minRadius=1, maxRadius=35)

        #if circles are found
        if circles is not None:
            #draw circles on the image
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                #check if the radius of the detected object (ball) is within the defined limits
                if self.min_radius <= i[2] <= self.max_radius:
                    #draw the circle on the image
                    cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)

                    #publish the position of the ball as a Point message
                    ball_position_msg = Point()
                    ball_position_msg.x = float(i[0])
                    ball_position_msg.y = float(i[1])
                    ball_position_msg.z = float(i[2])  #store the radius of the ball in the z-coordinate
                    self.ball_position_publisher.publish(ball_position_msg)
                    break  #stop processing after finding the first ball

        #convert the modified image back to ROS Image message
        try:
            image_with_circles_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        #publish the modified image with detected circles
        self.image_publisher.publish(image_with_circles_msg)

def main(args=None):
    rclpy.init(args=args)
    ball_detection_node = BallDetectionNode()
    rclpy.spin(ball_detection_node)
    ball_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
