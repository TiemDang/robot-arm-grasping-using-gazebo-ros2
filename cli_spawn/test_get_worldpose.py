import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import WorldPose

class GazeboPose(Node):
    def __init__(self):
        super().__init__('gazebo_pose')
        self.sub = self.create_subscription(
            WorldPose,
            '/world/default/pose/info',
            self.callback,
            10
        )

    def callback(self, msg):
        for p in msg.entities:
            if p.name == "robot::gripper_link":   # hoặc đúng tên entity
                pos = p.pose.position
                print("x =", pos.x, "y =", pos.y, "z =", pos.z)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
