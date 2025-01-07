import sys

from robot_interfaces.srv import StartStop
import rclpy
from rclpy.node import Node


class NodeMgmtClientAsync(Node):

    def __init__(self):
        super().__init__('node_mgmt_client_async')
        self.cli = self.create_client(StartStop, 'node_start_stop')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('node_start_stop service not available, waiting again...')
        self.req = StartStop.Request()

    def send_request(self, node_name, action):
        self.req.node = node_name #"cartographer"
        self.req.action = action #StartStop.Request().START
        self.get_logger().info('--req.action: %d' %(self.req.action))
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    node_mgmt_client = NodeMgmtClientAsync()
    future = node_mgmt_client.send_request(str(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(node_mgmt_client, future)
    response = future.result()
    node_mgmt_client.get_logger().info(
        ' the service node_start_stop, node_name:%s, action: %d, result:%d' %(str(sys.argv[1]), int(sys.argv[2]),response.success))

    node_mgmt_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()