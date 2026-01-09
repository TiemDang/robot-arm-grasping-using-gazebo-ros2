import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointRelay(Node):
    def __init__(self):
        super().__init__('joint_relay')
        self.sub = self.create_subscription(
            JointState,
            '/world/empty/model/my_robot/joint_state',
            self.callback,
            10
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

