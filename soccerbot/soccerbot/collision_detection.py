import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from soccer_sim.msg import Pose

agents = [
    'player_A1',
    'player_A2',
    'player_A3',
    'player_B1',
    'player_B2',
    'player_B3',
    'ball'
]

class CollisionDetection(Node):
    def __init__(self):
        super().__init__('collision_detection')
        self.subs = [self.create_subscription(Pose, f"/{agent}/pose", self.pose_sub_callback, 10) for agent in agents]
        self.publisher_ = self.create_publisher(Twist, '/ball/cmd_vel', 10)
        self.agent_positions = {}

    def pose_sub_callback(self, msg):
        # TODO:
        # Keep track of all positions, and see if the ball collides with a player
        # If it does, 'bounce' the ball by setting its velocity
        # Might also check if the ball goes into the goal 
        return

def main():
    rclpy.init()

    collision_detection = CollisionDetection()

    rclpy.spin(collision_detection)

    rclpy.shutdown()

if __name__ == '__main__':
    main()