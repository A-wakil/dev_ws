import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class TeleopTwistToAckermann(Node):
    def __init__(self):
        super().__init__('teleop_twist_to_ackermann')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/cmd_ackermann', 10)
        self.get_logger().info("Teleop Twist to Ackermann node started")

    def listener_callback(self, msg):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.steering_angle = msg.angular.z
        ackermann_msg.drive.speed = msg.linear.x
        self.publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
