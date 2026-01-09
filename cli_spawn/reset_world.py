import rclpy
from rclpy.node import Node
import subprocess
import time

class ResetWorldRobot(Node):
    def __init__(self):
        super().__init__('reset_world_robot')
        self.world_name = 'default'
        self.robot_name = 'my_robot'

        # Reset world
        self.reset_world()

        # Đợi 1s cho ổn
        time.sleep(1.0)

        # Đưa robot về vị trí ban đầu
        self.reset_robot_pose()

    def reset_world(self):
        self.get_logger().info('Resetting world...')
        cmd = [
            'ros2', 'service', 'call',
            f'/world/{self.world_name}/control',
            'gz_msgs/srv/WorldControl',
            "{world_control: {reset: {all: true}, pause: false}}"
        ]
        subprocess.run(cmd)
        self.get_logger().info('World reset done.')

    def reset_robot_pose(self):
        self.get_logger().info('Resetting robot pose...')
        cmd = [
            'ros2', 'service', 'call',
            f'/world/{self.world_name}/set_pose',
            'gz_msgs/srv/SetEntityPose',
            f"{{entity: {{name: '{self.robot_name}'}}, pose: {{position: {{x: 0.0, y: 0.0, z: 0.1}}, orientation: {{w: 1.0}}}}}}"
        ]
        subprocess.run(cmd)
        self.get_logger().info('Robot pose reset done.')

def main(args=None):
    rclpy.init(args=args)
    node = ResetWorldRobot()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

