import rclpy
from rclpy.node import Node
from rclpy import init, spin
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose

class SpawnPublisher(Node):
    def __init__(self):
        super().__init__("spawn_pub")

        self.pub = self.create_publisher(
            EntityFactory,
            "/world/default/create",
            10
        )

        msg = EntityFactory()
        msg.name = "my_robot"
        msg.allow_renaming = False
        msg.sdf_filename = "/home/venus/ros2_ws/src/cli_spawn/urdf/my_robot.urdf"

        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        
        msg.pose = pose

        self.pub.publish(msg)
        self.get_logger().info("Spawn sent")


def main():
    rclpy.init()
    node = SpawnPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
