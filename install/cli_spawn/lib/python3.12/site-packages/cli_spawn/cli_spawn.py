import rclpy
from rclpy.node import Node
import subprocess
import os

class MultiSpawn(Node):
    def __init__(self):
        super().__init__('multi_spawn')

        # Declare params
        self.declare_parameter('num_robots', 1)
        self.declare_parameter('model_path', '/home/venus/ros2_ws/src/cli_spawn/urdf/my_robot.xacro.sdf')

        num_robots = self.get_parameter('num_robots').value
        model_path = self.get_parameter('model_path').value

        # Spawn
        for i in range(num_robots):
            name = f"bot_{i}"
            self.spawn_entity(name, model_path, i)

    def spawn_entity(self, name, model_path, index):
        """Spawn using gz_sim_create not SpawnEntity service"""
        try:
            subprocess.run([
                "ros2", "run", "ros_gz_sim", "create",
                "-name", name,
                "-file", model_path,
                "-x", str(index * 2.0), 
                "-y", "0.0",
                "-z", "0.85"
            ], check=True)
            self.get_logger().info(f"Spawned {name} successfully!")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Spawned {name}: {e}")

def main():
    rclpy.init()
    node = MultiSpawn()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

