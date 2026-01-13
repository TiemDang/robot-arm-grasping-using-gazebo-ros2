
# Project Title
Robotic Arm control for grasping an object at a fixed location in Gazebo.



# Overview
This project simulates a robotic arm grasping an object in a simulation environment
using Gazebo and ROS 2. A Genetic Algorithm (GA) and Neural Network (NN) are applied to train the robot
to learn how to grasp the object autonomously.

# Objectives
The objectives of this project :
- To integrate a robotic arm into a simulation environment.
- To control the robotic arm using ROS 2 and Gazebo.
- To apply a Genetic Algorithm (GA) and a Neural Network (NN) to enable the robot to grasp a specific object at a fixed location.
# Idea
- The system consists of a robotic arm simulated in Gazebo and controlled through ROS 2.
  A Neural Network (NN) is used as the robot controller, which receives state information
  from the simulation environment and outputs control commands for the robotic arm.

- Instead of using gradient-based backpropagation, a Genetic Algorithm (GA) is employed
  to optimize the Neural Network weights. In this approach, each individual in the GA
  population represents a complete set of NN weights.

- During each generation, the NN controller corresponding to an individual is executed
  in the simulation environment. The robot’s behavior is evaluated using a fitness
  function based on its ability to grasp an object at a fixed location. Based on the
  fitness values, the GA performs selection, crossover, and mutation to evolve the
  population toward improved grasping performance.

- This process iteratively updates the NN weights through evolutionary optimization
  until a satisfactory grasping behavior is achieved or a stopping criterion is met.

# System Requirements
- Operating System: Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic
- Python
- ROS 2 packages for robot simulation and control
- NumPy ( For Genetic Algorithm and Neural Network)
# Usage
## 1. Clone the repository
```bash
git clone https://github.com/TiemDang/robot-arm-grasping-using-gazebo-ros2.git
```
Move directory robot-arm-grasping-using-gazebo-ros2 into the src directory of your ROS2 workspace
```bash
mv robot-arm-grasping-using-gazebo-ros2 ~/<your_ros2_workspace>/src/
```


## 2. Modify some file.
## 2.1 Declare where to save training results.
```bash 
cd ~/<your_ros2_workspace>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/
```
- Open file ga_calc.py
- Modify the following line to specify the directory for saving training results:
```bash 
 self.save_dir = " <your/save/path> "
``` 


## 2.2 Declare path to load training results.
```bash
cd ~/<your_ros2_workspace_name>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/
```
- Open file test_control.py
- Modify the following line to specify the path to trained model file: 
```bash
self.file_name = "<your/save/path/model_name.pkl>"
```
- Note : I have some trained file save in models directory. If you want to run it instead of training :
```bash
cd ~/<your_ros2_workspace_name>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/models/
pwd
```
- Copy result of pwd command (example : your/path/to/folder/models)
- Open file test_control.py and change to :
```bash
self.file_name = "your/path/to/folder/models/best_4.pkl>"
```
- If you want to change model, replace best_4.pkl to another .pkl file.


## 2.3 Some change to robot urdf file.
- Get full path to the meshes folder :
```bash
cd ~/<your_ros2_workspace_name>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/meshes/
pwd
```
- Copy results of pwd command (example : your/path/to/folder/meshes )
- Then run this command :
```bash
cd ~/<your_ros2_workspace_name>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/urdf/
```
- Open file my_robot.urdf
- Modify all lines start with : filename="file:///home/....... /<file_name.STL>
- Example like this line :
```bash
filename="file:///home/venus/ros2_ws/src/cli_spawn/meshes/base_link.STL" />
```
- Change to your path to .STL file on your computer :
```bash
filename="file:///your/path/to/folder/meshes/base_link.STL" />
```
- Save file
- Get full path to the config folder :
```bash
cd ~/<your_ros2_workspace_name>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/config/
pwd
```
- Copy results of pwd command (example : your/path/to/folder/config )
- Then :
```bash
cd ~/<your_ros2_workspace_name>/src/robot-arm-grasping-using-gazebo-ros2/cli_spawn/urdf/
```
- Open file my_robot.urdf
- Modify this line :
```bash
<parameters>/home/venus/ros2_ws/src/cli_spawn/config/myrobot_control.yaml</parameters>
```
- Change to this :
```bash
<parameters>/your/path/to/folder/config/myrobot_control.yaml</parameters>
```
- Save file


## 3. Build the ROS 2 workspace
```bash
source /opt/ros/<ros2_version>/setup.bash
cd <ros2_workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cli_spawn
source install/setup.bash
```


## 4. Launch the simulation world
```bash
ros2 launch cli_spawn empty_world.launch.py 
```
This will start the Gazebo simulation with the robotic arm loaded into the environment.


## 5. Run the learning process
### 5.1 Bridge parameter
Open new terminal
```bash
source /opt/ros/<ros2_version>/setup.bash
ros2 run ros_gz_bridge parameter_bridge /world/default/control@ros_gz_interfaces/srv/ControlWorld /world/default/set_pose@ros_gz_interfaces/srv/ControlWorld
```


### 5.2 Run the Genetic Algorithm node.
Open new terminal
```bash
cd <your_ros2_workspace_name>
source install/setup.bash
ros2 run <packages_name> ga_calc
```
This node evaluates each individual in the Genetic Algorithm population and evolves new generations.
Each individual corresponds to a set of joint angle values for the robotic arm used to perform object grasping.


### 5.3 Run the end position reader node
Open new terminal
```bash
cd <your_ros2_workspace_name>
source install/setup.bash
ros2 run <packages_name> read_end_point
```
This node is responsible for reading the state of the robotic arm and the target object.
The collected data is published to a ROS 2 topic and read by the Genetic Algorithm node for fitness evaluation and generate new generations.


### 5.4 Run the robot control node.
Open new terminal
```bash
cd <your_ros2_workspace_name>
source install/setup.bash
ros2 run cli_spawn control_position
```
This node is responsible for evaluating individuals generated by the Genetic Algorithm node and publishing the corresponding joint angle values to the robot control topic.


### 5.5 Video demo training process.
![Training demo](results/video_result.mp4)


## 6. Test training results
Open new terminal
```bash
cd <your_ros2_workspace_name>
source install/setup.bash
ros2 run cli_spawn test_ctrl
```
This will visualize the trained grasping result in the Gazebo simulation.




# Results
![Grasping result](results/gif_result.gif)
# Note
The robot URDF file used in this project is not my original work.
It was obtained from the following YouTube video and is used for
educational and research purposes only:

- https://www.youtube.com/watch?v=jsSuntiQZUg
