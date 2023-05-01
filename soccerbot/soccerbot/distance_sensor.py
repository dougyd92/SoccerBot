import rclpy
from rclpy.node import Node

from soccerbot_interfaces.srv import DistanceReq
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

class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')
        self.srv = self.create_service(DistanceReq, 'get_distance', self.get_distance_callback)
        self.subs = [self.create_subscription(Pose, f"/{agent}/pose", self.pose_sub_callback, 10) for agent in agents]
        self.agent_positions = {}

    def get_distance_callback(self, req, resp):
        from_id = req.from_id
        to_id = req.to_id

        # TODO
        resp.dist_x = 0.0
        resp.dist_y = 0.0
        return resp

    def pose_sub_callback(self, msg):
        return

def main():
    rclpy.init()

    distance_sensor = DistanceSensor()

    rclpy.spin(distance_sensor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()