#Hand Gesture Control of Panda Robot

This repository presents a ROS2 package that allows a user to move the Franka Emika Panda robotic arm via hand gestures captured via webcam, as well as open and close the gripper. The gripper can be closed by bringing the index and thumb fingers together, and opened by bringing them apart.  

This project uses OpenCV and MediaPipe for hand detection, MoveIt Servo for controlling the manipulator, and Gazebo for simulation.  

The ```handtracker.py``` node calculates the hand's coordinates using OpenCV and MediaPipe. It also sends action goals to the gripper depending on the distance between the thumb and the index tips. The ```manipulate.py``` node normalizes the coordinates calculated by the previous node into a range that the manipulator can use. Appropriate velocities are then published as TwistStamped messages to the ```/servo_node/delta_twist_cmds``` topic, where servo_node calculates the corresponding joint velocities.  

Additionally, several alterations were made to the original URDF, SRDF and YAML configuration files of the Franka Emika Panda robotic arm to port it to Gazebo.  
