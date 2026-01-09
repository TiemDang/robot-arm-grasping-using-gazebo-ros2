import sys
import gz.transport13 as gz_transport
from ros_gz_interfaces.msg import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.stringmsg_pb2 import StringMsg 

def spawn_robot():
    # Node Gazebo Transport
    node = gz_transport.Node()

    # Chuẩn bị request
    req = EntityFactory()
    req.name = "my_robot"
    req.sdf_filename = "/home/venus/ros2_ws/src/my_robot/urdf/my_robot.sdf"
    req.allow_renaming = True

    # Pose
    req.pose.position.x = 0.0
    req.pose.position.y = 0.0
    req.pose.position.z = 1.0
    req.relative_to = "world"

    # Gửi request
    success, rep = node.request("/world/default/create", req,request_type=StringMsg, response_type=StringMsg, timeout=2000)
    print("Success:", success)
    if rep:
        print("Response:", rep)

def main():
    spawn_robot()

if __name__ == "__main__":
    sys.exit(main())
