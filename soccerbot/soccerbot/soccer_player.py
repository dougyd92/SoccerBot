import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from random import randint

class PlayerFormationController(Node):

    def __init__(self):
        super().__init__('player_formation_ctrl')
        self.publisher_ = self.create_publisher(Twist, '/agent1/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        velocity = Twist()

        direction = randint(1,4)
        if direction == 1:
            velocity.linear.x = 1.0
        elif direction == 2:
            velocity.linear.x = -1.0
        elif direction == 3:
            velocity.linear.y = 1.0
        elif direction == 4:
            velocity.linear.y = -1.0

        self.publisher_.publish(velocity)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PlayerFormationController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()