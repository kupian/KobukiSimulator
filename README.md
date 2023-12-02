# Kobuki Simulator

## Installation

- Move kobuki_unity to your ros2 workspace/src folder
- From your ros2 workspace, run `colcon build --packages-select kobuki_unity` to build the package

## Running

- The simulator can be used from inside the Unity Editor or by running the *build.x86_64* executable
- If using the editor, make sure to source ros2 first with `source /opt/ros2/humble/setup.bash` then run the Unity Editor from terminal.
- Once the simulator is running, the Kobuki should publish to */cone_positions* and be ready to accept DriveCommand messages from */ugrdv_kobuki/drive_command*
- After sourcing install/setup.bash, the test navigator node can be run with `ros2 run kobuki_unity navigator`

## /cone_positions

- Currently, the simulator is approximating the functionality of the ZED2 camera and YOLO model by publishing a string of tuples, each representing (x,y,distance,colour).
- x,y are given as proportions of screen width, e.g. (0.5,0.5) for a cone directly in the centre of the screen.
- distance is given as the length of the straight line between the Kobuki and cone, in metres.
- colour is given as a float, 0 for blue and 1 for yellow

## Changes

- Feel free to fork and change/add anything you like and create a pr
