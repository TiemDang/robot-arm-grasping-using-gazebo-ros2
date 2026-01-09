import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/venus/ros2_ws/src/cli_spawn/install/cli_spawn'
