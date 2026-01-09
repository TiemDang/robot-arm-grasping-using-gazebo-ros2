import pickle

filename = "/home/venus/ros2_ws/src/cli_spawn/models/best_4.pkl"

with open(filename, "rb") as f:
    data = pickle.load(f)

print(data) 
