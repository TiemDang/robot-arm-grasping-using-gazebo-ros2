import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from ament_index_python.packages import get_package_share_directory
import os

class ResetWorld(Node):
    def __init__(self):
        super().__init__('reset_world_client')

        self.cli = self.create_client(ControlWorld, '/world/default/control')
        #self.spawn_cli = self.create_client(SpawnEntity, '/world/default/create')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /world/default/control ...')

        req = ControlWorld.Request()

        # Path
        req.world_control.reset.all = True
        self.future = self.cli.call_async(req)



def main():
    rclpy.init()
    node = ResetWorld()
    rclpy.spin_until_future_complete(node, node.future)
    try :
        result = node.future.result()
        node.get_logger().info(f"Reset world done. Success={result.success}")
    except Exception as e:
        node.get_logger().error(f"Failed: {e}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

