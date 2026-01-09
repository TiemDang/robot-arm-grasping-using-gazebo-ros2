import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ros_gz_interfaces.srv import ControlWorld, SpawnEntity
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
import time
import numpy as np
from launch.actions import ExecuteProcess
import subprocess
import os

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        # Pub
        self.control_pub = self.create_publisher(Float64MultiArray,
                                         '/forward_position_controller/commands',
                                         10)
        self.state_pub = self.create_publisher(String, 'state_check', 10) # Update control state
        
        # Sub
        self.subscription = self.create_subscription(String, 'state_check', self.state_callback, 10) # Read state of reading end point
        self.ga_subscription = self.create_subscription(Float64MultiArray, 'ga_population', self.ga_callback, 10)
        
        # Reset client
        self.reset_client = self.create_client(ControlWorld, '/world/default/control')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /world/default/control ...')
            
        
        # Init values
        self.step = 0
        self.started = False
        self.population_received = False

        # Theta value for test position
        self.default_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.positions = []
        # Sending first time
        self.timer = self.create_timer(1.0, self.first_command)
    def first_command(self):
        if not self.started:
            self.get_logger().info('Sending default pose...')
            # Create msg for pose and state
            msg = Float64MultiArray()
            state_msg = String()

            # Data
            msg.data = self.default_pose
            state_msg.data = 'Public_Done'

            # Publish to topic
            self.control_pub.publish(msg)
            time.sleep(1)
            self.state_pub.publish(state_msg)

            # Terminate
            self.started = True
            self.timer.cancel()

    def reset_pose(self):
        req = ControlWorld.Request()
        
        # reset world
        req.world_control.reset.all = True
        future = self.reset_client.call_async(req)
        time.sleep(3)
        
        # spawn robot to default pose
        subprocess.run([
            "ros2", "launch", "cli_spawn", "robot_spawn.launch.py"
        ])
        self.get_logger().info("Reseting pose...")
        time.sleep(2)
        """
        reset_pose_msg = Float64MultiArray()
        reset_pose_msg.data = self.default_pose
        #Reset robot to default pose
        self.control_pub.publish(reset_pose_msg)
        self.get_logger().info("Reseting pose...")
        time.sleep(2)
        """

    def send_command(self):
        if not self.population_received or not self.positions: # Check if received GA pub or not
            self.get_logger().warn("No GA population available — skipping send_command.")
            return
        
        # State
        state_msg = String()
        state_msg.data = 'Public_Done'
        # Control value
        control_msg = Float64MultiArray()
        control_msg.data = self.positions[self.step]
        # Public value to control robot and update state
        self.control_pub.publish(control_msg)
        time.sleep(2) # delay time for robot moving to last cordinate completely
        self.state_pub.publish(state_msg)
        self.get_logger().info(f"Sent individual {self.step+1}/{len(self.positions)} : {self.positions[self.step]}")
        self.step +=1
        # If send all individual in generation
        if self.step >= len(self.positions):
            self.step = 0
            self.state_pub.publish(String(data='Generation Done'))
            self.get_logger().info("---> Generation Done, waiting for next gen from GA_calc.")
            self.population_received = False
            return
    
    def ga_callback(self, msg):
        num_joint = 7
        ga_data = np.array(msg.data)
        try :
            self.positions = ga_data.reshape(-1, num_joint).tolist()
            self.population_received = True
            self.step = 0
            self.get_logger().info(f"Received GA population with {len(self.positions)} individuals.")
        except ValueError as e:
            self.get_logger().error(f"Error reshaping GA data: {e}")
            self.positions = []
            self.population_received = False
        
    def state_callback(self, msg):
        if msg.data == 'Calculating_Done' or msg.data == 'Calculating_All_Gene_Done':
            self.reset_pose()
            self.send_command()

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

