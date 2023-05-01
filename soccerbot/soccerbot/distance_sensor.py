import rclpy
from rclpy.node import Node

from soccerbot_interfaces.srv import DistanceReq

class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')
        self.srv = self.create_service(DistanceReq, 'get_distance', self.get_distance_callback)

    def get_distance_callback(self, req, resp):
        from_id = req.from_id
        to_id = req.to_id

        # TODO
        resp.dist_x = 0.0
        resp.dist_y = 0.0
        return resp

def main():
    rclpy.init()

    distance_sensor = DistanceSensor()

    rclpy.spin(distance_sensor)

    rclpy.shutdown()

if __name__ == '__main__':
    main()