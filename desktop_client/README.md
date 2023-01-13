# Desktop Client
The desktop client uses RViz, the 3D visualization tool for ROS, to visualize the state of the robot and its environment. The program is ran on a computer connected to the same network as the robot, where multicast support is enabled.

## How to Run
1. Source the `local_setup.bash` file in the `/install` folder: `source local_setup.bash`
2. Run `launch.py` in the `/launch` folder: `ros2 launch launch.py`