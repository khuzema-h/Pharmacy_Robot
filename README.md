# Pharmacy Robot Instructions
Contains the robot description and controllers for simulation in ROS2 and Gazebo

### Build instructions

First, source your ROS 2 workspaces with all the required dependencies.
Then, you are ready to clone and build this repository.
You should only have to do this once per install.

```sh
mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/khuzema-h/Pharmacy_Robot.git
cd ..
rosdep install --from-path src --ignore-src -yi
colcon build
```

### Initialization instructions

You will have to do this in every new session in which you wish to use these examples:

```sh
source ~/dev_ws/install/local_setup.sh
```

### Launch Robot in World

After Building and Sourcing the Workspace, Launch the Pharmacy Robot:

```sh
ros2 launch pharmacy_robot gazebo_launch.py
```
## Run Scanner Node

Open a Separate Terminal Window after Launching the robot in Gazebo to run the Scanner to read QR codes:

```sh
ros2 run pharmacy_robot scanner.py
```
![image](https://github.com/user-attachments/assets/ddaca3d2-5588-45ae-a0d4-5ac14c74bd46)

Open a Separate Terminal Window to visualize the camera feed in Rviz.

Run rviz:

```sh
rviz2
```
Add -> By Topic -> image


### Launch the Gripper:

In a new gazebo world, launch the gripper using: 

```sh
ros2 launch robotiq_2f_140_gripper_visualization gazebo_launch.py
```
The robot may not show in gazebo. To visualize in Rviz and controll the gripper:

In a separate terminal run Rviz:

```sh
rviz2
```
In another terminal run:

```sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
In Rviz:
Add -> Robot Model 
In the side panel set the following parameters:
![image](https://github.com/user-attachments/assets/d1e3c82f-75c8-4ae0-8d54-6e87151feda0)





### Potential pitfalls

If you are unable to automatically install dependencies with rosdep (perhaps due to [this issue](https://github.com/ros-infrastructure/rosdep/issues/733)), please do be sure to manually install the dependencies for your particular example of interest, contained in its package.xml file.
