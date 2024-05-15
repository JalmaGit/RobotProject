import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from angle_msg.msg import Pitchroll
import math

dt = 0.1
g = 9.81

class BallSimulatorNode(Node):
    def __init__(self):
        super().__init__('ball_simulator')
        self.subscription = self.create_subscription(Pitchroll, 'pitch_roll_message', self.angle_callback, 10)
        self.publisher = self.create_publisher(PointStamped, 'sim_ball_position', 10)
        self.timer = self.create_timer(dt, self.publish_position)
        self.angle = Pitchroll()
        self.position = [0, 0]
        self.velocity = [0, 0]
        self.platform_radius = 20

    def angle_callback(self, msg):
        self.angle = msg

    def publish_position(self):
        angle_radians = [self.angle.roll, self.angle.pitch]
        self.simulate_position(angle_radians)
        self.publish_point(self.position)

    def simulate_position(self, angle):
        # Simulate ball position based on angle
    
        accelerationX = -g * math.sin(angle[0])
        self.velocity[0] += accelerationX * dt
        self.position[0] += self.velocity[0] * dt
        if self.position[0] >= self.platform_radius:
            self.position[0] = self.platform_radius
        
        if self.velocity[0] > 10:
            self.velocity[0] = 10
        
        if self.position[0] <= -self.platform_radius:
            self.position[0] = -self.platform_radius

        if self.velocity[0] < -10:
            self.velocity[0] = -10

        accelerationY = -g * math.sin(angle[1])
        self.velocity[1] += accelerationY * dt
        self.position[1] += self.velocity[1] * dt
        if self.position[1] >= self.platform_radius:
            self.position[1] = self.platform_radius
        
        if self.velocity[1] > 10:
            self.velocity[1] = 10
        
        if self.position[1] <= -self.platform_radius:
            self.position[1] = -self.platform_radius            
        
        if self.velocity[1] < -10:
            self.velocity[1] = -10

    def publish_point(self, position):
        point = PointStamped()
        point.header.frame_id = 'map'
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = float(position[0])  # Assign x-coordinate
        point.point.y = float(position[1])  # Assign y-coordinate
        print(f"pos={point} and vel={self.velocity}")
        self.publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = BallSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
