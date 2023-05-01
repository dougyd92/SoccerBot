import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist

from random import randint

from soccerbot_interfaces.srv import DistanceReq

"""
    Controller for soccer player bots.
    Implements a decentralized formation control algorithm.
"""
class PlayerFormationController(Node):

    def __init__(self):
        super().__init__('player_formation_ctrl')
        self.publisher_ = self.create_publisher(Twist, '/agent1/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

        self.distance_client = self.create_client(DistanceReq, '/get_distance', callback_group=None)
        while not self.distance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.dist_req = DistanceReq.Request()

    async def timer_callback(self):
        velocity = Twist()

        # TODO Implement formation control alg
        distance = await self.send_dist_request(1,2)
        self.get_logger().info(f"got distance to neighbor: {distance.dist_x},{distance.dist_y}")

        # Placeholder
        direction = randint(1,4)
        if direction == 1:
            velocity.linear.x = 4.0
        elif direction == 2:
            velocity.linear.x = -4.0
        elif direction == 3:
            velocity.linear.y = 4.0
        elif direction == 4:
            velocity.linear.y = -4.0

        self.publisher_.publish(velocity)

    async def send_dist_request(self, from_id, to_id):
        self.dist_req.from_id = from_id
        self.dist_req.to_id = to_id
        return await self.distance_client.call_async(self.dist_req)

def main(args=None):
    rclpy.init(args=args)

    formation_controller = PlayerFormationController()

    executor = MultiThreadedExecutor()
    executor.add_node(formation_controller)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()