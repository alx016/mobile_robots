## Maze Mapping using an Omnidirectional Mobile Robot (OMR)

Using an Omnidirectional Mobile Robot developed navigation algorithms so that the 
robot would be able to avoid obstacles across a Maze until it finds a specific target. 
For this project the target were 3 soda cans. Once the robot found them, it would 
use its robotic arm to reach for one of those cans and take it back to the starting position.

### Softwares and specifics:
* Robot Operating System (ROS)
  * ROS melodic
* Ubuntu 18.04
* Python 3.6

### Algorithms:
* RRT* for Path Planning (used to return to the starting point)
* Inverse Kinematics and Direct Kinematics for the robotic arm
* For obstacle avoidance, the algorithm used was for the robot to
  follow the obstacle (wall) at its right.
* For obstacle identification a Convolutional Neuroal Network (CNN)
  was trained with the help of RoboFlow and Yolo. Later it was
  optimized into a .ONNX file for faster processing. 
  




### Results:
[![Maze mapping](https://img.youtube.com/vi/Q_rxTU3xUOk/0.jpg)](https://www.youtube.com/watch?v=Q_rxTU3xUOk)

https://youtu.be/Q_rxTU3xUOk
