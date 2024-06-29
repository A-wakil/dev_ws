import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/ackermann_controller/cmd_vel_unstamped', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd = Twist()

    def timer_callback(self):
        self.cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd.angular.z = 0.1  # Turn at 0.1 rad/s
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
