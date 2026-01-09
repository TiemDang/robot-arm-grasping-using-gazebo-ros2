import subprocess
import re
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import math

    
def error_cal():
    x_tar = -0.03
    y_tar = -0.081
    z_tar = 0.641
    ox_tar = 0.705
    oy_tar = 0.053
    oz_tar = 0.705
    ow_tar = 0.053
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
                    
    coords = {k: round(v, 3) for k, v in coords.items()}
    error = math.sqrt((coords["x"] - x_tar)**2 + (coords["y"] - y_tar )**2 + (coords["z"] - z_tar)**2
                       + (coords["ox"] - ox_tar)**2 + (coords["oy"] - oy_tar)**2 + (coords["oz"] - oz_tar)**2 + (coords["ow"] - ow_tar)**2)
    print(f'error: {error:.3f}')
                    
while True:
    error_cal()
    time.sleep(1)
    
    
