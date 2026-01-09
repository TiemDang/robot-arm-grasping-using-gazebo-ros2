#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import pickle
from cli_spawn.ClassNeuralNetwork import NeuralNet


class JointPositionCommander(Node):
    def __init__(self):
        super().__init__('joint_position_commander')

        # topic controller in yaml
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        self.load_timer = self.create_timer(3.0, self.timer_callback)
        self.file_name = "/home/venus/ros2_ws/src/cli_spawn/models/best_4.pkl" # model path
        self.position = None
        self.crab_position = [
            [1.5, -1.0, 0.0, -1.1, 0.0, 0.0, 0.0],
            [1.5, 0.95, 0.0, -1.1, 0.0, 0.0, 0.0],
        ]
        self.index = 0
    def load_model(self):
        with open(self.file_name, "rb") as f:
            model_data = pickle.load(f)
        x = [3.424, 0.05, 1.157, 0.707, round(-3.04020054805064e-08, 3), 0.707,
             round(-2.6900923335165718e-06, 3), -0.03, -0.081, 0.641, 0.705, 0.503,
             0.705, 0.503]
        w_loaded = model_data["w"]
        v_loaded = model_data["v"]
        nn = NeuralNet(x, w_loaded, v_loaded)
        output = nn.FeedForward()
        rounded_output = [round(float(x), 3) for x in output]
        self.position = rounded_output + [0.0, 0.03, -0.03]
        msg = Float64MultiArray()
        msg.data = self.position
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent joint positions: {msg.data} | load from model')
        return rounded_output

    def crab_object(self, rounded_output):
        msg = Float64MultiArray()
        #self.crab_position[0] = rounded_output[:4] + self.crab_position[0][4:]
        msg.data = self.crab_position[self.index]
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent joint positions: {msg.data} | crab')
        self.index = (self.index + 1) % len(self.crab_position)

    def timer_callback(self):
        output = self.load_model()
        time.sleep(1)
        self.crab_object(output)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
"""
[0.0, 0.95, 0.0, -1.1, 0.0, 0.03, -0.03] # target,
[1.5, -1.0, 0.0, -1.1, 0.0, 0.0, 0.0],
[1.5, 0.95, 0.0, -1.1, 0.0, 0.0, 0.0],
[1.5, 0.95, 0.0, -1.1, 0.0, 0.03, -0.03],
[1.5, 0.95, 0.0, -1.1, 0.0, 0.0, 0.0],
[1.5, -1.0, 0.0, -1.1, 0.0, 0.0, 0.0],
[0.0, 0.95, 0.0, -1.1, 0.0, 0.0, 0.0]
"""
