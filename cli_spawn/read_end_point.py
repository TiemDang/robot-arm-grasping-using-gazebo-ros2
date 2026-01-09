import subprocess
import re
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
class Read_end_point(Node):
    def __init__(self):
        super().__init__('read_end_point')
        # Create publish and subscribe
        self.pose_pub = self.create_publisher(Float64MultiArray, 'endpoint_pose', 10)
        self.state_pub = self.create_publisher(String, 'state_check', 10)
        self.state_sub = self.create_subscription(String, 'state_check', self.state_callback, 10)
    def state_callback(self, msg):
        if msg.data == 'Public_Done':
            proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )

            # Flag
            capture = False
            in_position = False
            in_orientation = False
            coords = {}

            for line in proc.stdout:
                line = line.strip()
                # link_6
                if 'name: "link_6"' in line:
                    capture = True
                    coords = {}
                    continue

                if capture:
                    if line.startswith("position {"):
                        in_position = True
                        continue
                    elif line.startswith("orientation {"):
                        in_position = False
                        in_orientation = True
                        continue
                    elif line == "}":
                        if in_position:
                            in_position = False
                            continue
                        elif in_orientation:
                            in_orientation = False
                            continue
                        else:
                            required_keys = ["x","y","z","ox","oy","oz","ow"]
                            if all(k in coords for k in required_keys):
                                print(f"x={coords['x']}, y={coords['y']}, z={coords['z']}, "
                                      f"ox={coords['ox']}, oy={coords['oy']}, oz={coords['oz']}, ow={coords['ow']}")
                                self.publish_pose(coords)
                                self.update_state()
                                capture = False
                                proc.terminate()
                                break
                    # Parse value
                    match = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                    if not match:
                        continue
                    val = float(match[0])
                    
                    if in_position:
                        if line.startswith("x:"):
                            coords["x"] = val
                        elif line.startswith("y:"):
                            coords["y"] = val
                        elif line.startswith("z:"):
                            coords["z"] = val
                    
    
                    # Orientation block
                    elif in_orientation:
                        if line.startswith("x:"):
                            coords["ox"] = val
                        elif line.startswith("y:"):
                            coords["oy"] = val
                        elif line.startswith("z:"):
                            coords["oz"] = val
                        elif line.startswith("w:"):
                            coords["ow"] = val
                    
    def publish_pose(self, coords):
        pose_msg = Float64MultiArray()
        pose_msg.data = [
            round(coords["x"], 3),
            round(coords["y"], 3),
            round(coords["z"], 3),
            round(coords["ox"], 3),
            round(coords["oy"], 3),
            round(coords["oz"], 3),
            round(coords["ow"], 3)
        ]
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"Published pose: pos=({round(coords['x'],3)},{round(coords['y'],3)},{round(coords['z'],3)}), "
            f"ori=({round(coords['ox'],3)},{round(coords['oy'],3)},{round(coords['oz'],3)},{round(coords['ow'],3)})"
        )
    def update_state(self):
        state_msg = String()
        state_msg.data = 'Reading_Done'
        self.state_pub.publish(state_msg)
        self.get_logger().info('Reading done...Waiting for calculating ')

def main(args=None):
    rclpy.init(args=args)
    node = Read_end_point()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
